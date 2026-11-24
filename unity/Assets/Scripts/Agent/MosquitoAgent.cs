using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace MosquitoSim
{
    /// <summary>
    /// Subgoal-conditioned mosquito agent (interface v3).
    /// Behavior Parameters: name "Mosquito", vector obs 59, continuous actions 3.
    /// Decision Requester: period 5, take actions between decisions on.
    ///
    /// Every fixed step (0.01 s): move, update human and hand (stage 2+), check
    /// failure, shape reward toward the current command, track subgoal end,
    /// count feeding, then let the planner decide if an event is pending.
    /// Planner events: episode_start, subgoal_achieved, subgoal_timeout,
    /// alert_start, alert_end, hand_active; at most one decision per 1 s.
    /// </summary>
    [RequireComponent(typeof(MosquitoMotor))]
    public class MosquitoAgent : Agent
    {
        [Tooltip("Episode length in simulated seconds; sets MaxStep if it is 0")]
        public float episodeSeconds = 60f;
        public LayerMask rayMask = ~(1 << 9);

        const float k_ProgressCoef = 0.1f;
        const float k_CollisionPenalty = 0.01f;
        const float k_AchieveBonus = 0.3f;
        const float k_MinDecisionInterval = 1f;
        const float k_MidSwitchProb = 0.01f; // per decision step: about once per 5 s
        const int k_DecisionPeriod = 5;

        MosquitoMotor m_Motor;
        RoomArea m_Area;
        readonly float[] m_Rays = new float[ObsLayout.NumRays];
        readonly float[] m_Obs = new float[ObsLayout.ObsSize];
        readonly SubgoalTracker m_Tracker = new SubgoalTracker();
        readonly HashSet<string> m_Excluded = new HashSet<string>();
        readonly List<string> m_Events = new List<string>();
        readonly Dictionary<int, List<Candidate>> m_Snapshots = new Dictionary<int, List<Candidate>>();

        EnvParams m_P;
        int m_Stage;
        System.Random m_Rng;
        int m_Seed;
        int m_EpisodeIndex = -1;
        string m_EpisodeId;

        SkinZone m_TargetSkin;
        float m_SimTime, m_LastDecisionTime, m_FeedTimer;
        int m_DecisionIndex;
        bool m_PrevAlert, m_PrevHandActive;
        bool m_Succeeded, m_Caught, m_Trapped, m_HasEpisode;
        int m_Collisions;

        // Direct subgoal check (planner = Probe): one subgoal per episode, 8 s.
        const float k_HideStartMin = 0.5f, k_HideStartMax = 1.0f; // HIDE scenario: start this far from the head
        const float k_HideMaxWait = 10f;                           // HIDE scenario: no valid choice by then -> search with the mosquito held
        const float k_HideMaxTravel = 8f;                          // HIDE scenario: search starts anyway if the spot is not reached by then
        const float k_ProbeHandMin = 0.4f, k_ProbeHandMax = 0.6f;  // EVADE: start this far from the hand
        const float k_ProbeSwingDelay = 0.5f;                       // EVADE: the hand swings after this
        bool m_Probe, m_ProbeStarted, m_ProbeDone, m_ProbeTargetHidden, m_ProbePathBlocked;
        // HIDE scenario (probe HIDE, or planner evaluation with hide_scenario): frozen until a HIDE choice,
        // then the human turns toward the buzzing and looks around; spotted = fail, alert over = success.
        bool m_HideScenario, m_Chosen, m_Searching, m_ProbeValid, m_ProbeArrived;
        float m_WaitStart, m_TravelStart, m_SearchStart, m_ProbeWait, m_ProbeTravel;
        Vector3 m_SoundPos;
        Subgoal m_ProbeSubgoal;
        string m_ProbeTarget, m_ProbeOutcome;
        float m_ProbeTime, m_ProbeStartDist, m_ProbeMinDist, m_ProbeHold, m_ProbeTargetHeight;

        public PlannerCommand CurrentCommand => m_Tracker.Command;
        public float SimTime => m_SimTime;
        public Vector3 SubgoalTarget => m_Tracker.Target;
        /// <summary>Point the policy is steered to: a route waypoint for MOVE_TO / HIDE, else the target.</summary>
        public Vector3 GuideTarget => m_Guide.Active ? m_Guide.Waypoint : m_Tracker.Target;
        readonly PathGuide m_Guide = new PathGuide();
        bool PointSubgoal => m_Tracker.Command.Subgoal == Subgoal.MoveTo || m_Tracker.Command.Subgoal == Subgoal.Hide;
        bool Guided => m_P.PathGuide && (m_Tracker.Command.Subgoal == Subgoal.MoveTo || m_Tracker.Command.Subgoal == Subgoal.Hide);
        public float FeedTimer => m_FeedTimer;
        public bool Landed => m_Motor != null && m_Motor.IsLanded;
        public Vector3 Velocity => m_Motor != null ? m_Motor.Velocity : Vector3.zero;
        public int Stage => m_Stage;
        public RoomArea Area => m_Area;
        public float[] LastObservation => m_Obs;

        /// <summary>Episode outcome counts across all agents (read by AutoTestRunner).</summary>
        public static int Episodes, Successes, Caught, Trapped, Decisions;
        /// <summary>Auto-test report: guided steps, and those without a route (fell back to straight line).</summary>
        public static int GuidedSteps, UnroutedSteps;
        /// <summary>Direct subgoal checks per subgoal wire name: trials, achieved, seconds to achieve.</summary>
        public static readonly Dictionary<string, Vector3> ProbeStats = new Dictionary<string, Vector3>();
        /// <summary>Auto-test report: "SUBGOAL/outcome" -> (count, summed seconds).</summary>
        public static readonly Dictionary<string, Vector2> ProbeOutcomes = new Dictionary<string, Vector2>();
        /// <summary>Auto-test report: first HIDE episodes in detail.</summary>
        public static readonly List<string> HideTrace = new List<string>();
        float m_SearchAngle;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void ResetCounters() { Episodes = Successes = Caught = Trapped = Decisions = GuidedSteps = UnroutedSteps = 0; ProbeStats.Clear(); ProbeOutcomes.Clear(); HideTrace.Clear(); }

        public override void Initialize()
        {
            m_Motor = GetComponent<MosquitoMotor>();
            m_Area = GetComponentInParent<RoomArea>();
            if (MaxStep == 0)
                MaxStep = Mathf.RoundToInt(episodeSeconds / Time.fixedDeltaTime);
        }

        public override void OnEpisodeBegin()
        {
            RecordPreviousEpisode();

            m_P = EnvParams.Read();
            m_Tracker.HideHoldSeconds = m_P.HideHold;
            m_Tracker.MoveToRadius = m_P.MoveToRadius;
            m_EpisodeIndex++;
            m_Seed = m_P.IsTraining
                ? unchecked(System.Environment.TickCount * 31 + m_Area.roomIndex * 7919 + m_EpisodeIndex)
                : EpisodeSeed.Make(m_P.EvalSeed, m_P.Split, m_Area.roomIndex, m_EpisodeIndex);
            m_Rng = new System.Random(m_Seed);
            m_Stage = m_P.Stage;
            if (m_P.IsTraining && m_Stage > 0 && m_Rng.NextDouble() < m_P.PrevStageMix) m_Stage--;
            m_EpisodeId = string.Format("{0}/s{1}/r{2:00}e{3:0000}", EnvParams.SplitName(m_P.Split), m_P.EvalSeed,
                m_Area.roomIndex, m_EpisodeIndex);

            m_Area.ResetEpisode(m_P, m_Stage, m_Rng);
            m_Motor.ResetMotor(m_Area.SampleFreePoint(m_Rng, 1.0f));
            SetUpProbe();

            m_SimTime = 0f;
            m_FeedTimer = 0f;
            m_DecisionIndex = 0;
            m_Succeeded = m_Caught = m_Trapped = false;
            m_Collisions = 0;
            m_PrevAlert = m_PrevHandActive = false;
            m_Excluded.Clear();
            m_Snapshots.Clear();
            m_Events.Clear();
            m_HasEpisode = true;

            // Until the first answer arrives the mosquito approaches the nearest skin.
            SetCommand(Planners.Fixed(Context()), null);
            m_Events.Add("episode_start");
            if (m_HideScenario) m_Events.Add("hide_scenario");
            m_LastDecisionTime = -999f;
            Decide(false);
        }

        // ---------------------------------------------------------------- observations

        public override void CollectObservations(VectorSensor sensor)
        {
            Vector3 pos = transform.position;
            ObsBuilder.CastRays(pos, rayMask, m_Rays);
            bool dynamic = m_Stage >= 2;
            var human = m_Area.human;
            var hand = m_Area.hand;
            var o = new ObsInput
            {
                Velocity = m_Motor.Velocity,
                // Experiment (11/24): while the subgoal is a point (MOVE_TO / HIDE) the skin-target slots
                // carry the subgoal target too. The policy steers along the sum of the two targets, so a
                // stale skin there pulls it away from the point (docs/architecture.md, 11/24).
                TargetSkinRel = (PointSubgoal ? GuideTarget
                    : m_TargetSkin != null ? m_TargetSkin.Center : m_Tracker.Target) - pos,
                SubgoalTargetRel = GuideTarget - pos,
                HumanVelocity = dynamic ? human.Velocity : Vector3.zero,
                HumanAlert = dynamic && human.Alert,
                HandActive = dynamic && hand.Active,
                HandRel = dynamic ? hand.Position - pos : Vector3.zero,
                HandVelocity = dynamic ? hand.Velocity : Vector3.zero,
                // Mosquito coil (stage 3, extension) is not implemented: hazard fields stay 0.
                Subgoal = m_Tracker.Command.Subgoal,
                RemainingTime01 = MaxStep > 0 ? 1f - (float)StepCount / MaxStep : 1f,
                Landed = m_Motor.IsLanded,
            };
            ObsBuilder.Build(o, m_Rays, m_Obs);
            sensor.AddObservation(m_Obs);
        }

        // ---------------------------------------------------------------- step

        public override void OnActionReceived(ActionBuffers actions)
        {
            var a = actions.ContinuousActions;
            var action = new Vector3(Mathf.Clamp(a[0], -1f, 1f), Mathf.Clamp(a[1], -1f, 1f), Mathf.Clamp(a[2], -1f, 1f));
            float dt = Time.fixedDeltaTime;
            m_SimTime += dt;

            // Apply commands that arrived from Python since the last step.
            PollExternal();

            if (m_HideScenario && !m_Chosen)
            {
                // Waiting for the HIDE choice: mosquito and human hold still.
                if (m_SimTime - m_WaitStart > k_HideMaxWait) ChooseHide(null);
                else return;
            }

            var cmd = m_Tracker.Command;
            SkinZone landing = cmd.Subgoal == Subgoal.Approach && m_TargetSkin != null && !m_TargetSkin.covered
                ? m_TargetSkin : null;
            // HIDE scenario without a valid choice: the mosquito stays where it is.
            bool hold = m_HideScenario && !m_ProbeValid;
            MotorStepResult r = hold ? default(MotorStepResult) : m_Motor.Step(action, dt, landing, m_P.LandingRadius, m_P.LandingSpeed);
            Vector3 pos = transform.position;

            if (r.HitTrap)
            {
                m_Trapped = true;
                AddReward(-1f);
                EndEpisode();
                return;
            }
            if (r.Collided)
            {
                AddReward(-k_CollisionPenalty);
                m_Collisions++;
            }
            AddReward(-m_P.TimePenalty * dt);

            // Human and hand
            var human = m_Area.human;
            var hand = m_Area.hand;
            if (m_HideScenario && m_Stage < 2) human.Step(dt, pos, m_Motor.IsLanded); // the search runs at any stage
            if (m_Stage >= 2)
            {
                human.Step(dt, pos, m_Motor.IsLanded);
                if (hand.Step(dt, pos, m_Motor.Velocity))
                {
                    m_Caught = true;
                    AddReward(-1f);
                    EndEpisode();
                    return;
                }
                if (human.Alert && !m_PrevAlert) m_Events.Add("alert_start");
                if (!human.Alert && m_PrevAlert) m_Events.Add("alert_end");
                if (hand.Active && !m_PrevHandActive) m_Events.Add("hand_active");
                m_PrevAlert = human.Alert;
                m_PrevHandActive = hand.Active;
            }

            if (m_HideScenario && !m_ProbeDone)
            {
                float d = Vector3.Distance(pos, m_Tracker.Target);
                if (m_ProbeValid) m_ProbeMinDist = Mathf.Min(m_ProbeMinDist, d);
                if (!m_Searching)
                {
                    // The mosquito flies to its spot while the human stands still; the alert starts on arrival.
                    bool arrived = m_ProbeValid && d <= SubgoalConstants.HideRadius;
                    if (arrived || !m_ProbeValid || m_SimTime - m_TravelStart > k_HideMaxTravel) BeginHideSearch(arrived);
                }
            }
            if (m_HideScenario && m_Searching && !m_ProbeDone)
            {
                string outcome = human.Spotted ? "spotted" : !human.Alert ? "achieved" : null;
                if (outcome != null)
                {
                    FinishProbe(outcome);
                    EndEpisode();
                    return;
                }
            }

            // Subgoal shaping and end
            m_Tracker.UpdateTarget(ResolveTarget(m_Tracker.Command, null));
            float progress = UpdateGuide(pos);
            if (!m_Motor.IsLanded) AddReward(k_ProgressCoef * progress);
            if (m_P.HideHoldReward > 0f && m_Tracker.Holding) AddReward(m_P.HideHoldReward * dt);
            bool hazard = m_Stage >= 2 && hand.Active;
            SubgoalStatus status = m_Tracker.Step(m_SimTime, dt, pos, m_Motor.IsLanded, hazard);
            if (status != SubgoalStatus.Running && m_P.Planner == PlannerKind.Random)
            {
                // Training progress per subgoal (TensorBoard). Mid-switched subgoals are not counted.
                string n = PlannerCommand.WireName(m_Tracker.Command.Subgoal);
                var stats = Academy.Instance.StatsRecorder;
                stats.Add("Subgoal/" + n + "/Achieved", status == SubgoalStatus.Achieved ? 1f : 0f);
                if (status == SubgoalStatus.Achieved) stats.Add("Subgoal/" + n + "/Seconds", m_SimTime - m_Tracker.StartTime);
            }
            if (m_Probe && !m_HideScenario && m_ProbeStarted && !m_ProbeDone && JudgeProbe(dt, pos, status))
            {
                EndEpisode();
                return;
            }
            if (status == SubgoalStatus.Achieved)
            {
                if (m_Tracker.Command.Subgoal != Subgoal.Approach) AddReward(m_P.AchieveBonus);
                m_Events.Add("subgoal_achieved");
            }
            else if (status == SubgoalStatus.TimedOut)
            {
                if (m_Tracker.Command.Subgoal == Subgoal.Approach && m_TargetSkin != null)
                    m_Excluded.Add(m_TargetSkin.id);
                m_Events.Add("subgoal_timeout");
            }

            // Feeding
            if (r.TookOff) m_FeedTimer = 0f;
            bool feedGated = m_P.FeedGate && m_Tracker.Command.Subgoal != Subgoal.Approach;
            if (feedGated) m_FeedTimer = 0f;
            else if (m_Motor.IsLanded && m_Motor.LandedZone != null && !m_Motor.LandedZone.covered)
            {
                m_FeedTimer += dt;
                if (m_FeedTimer >= m_P.FeedTime)
                {
                    m_Succeeded = true;
                    AddReward(1f);
                    EndEpisode();
                    return;
                }
            }

            bool decisionStep = StepCount % k_DecisionPeriod == 0;
            Decide(decisionStep);
        }

        // ---------------------------------------------------------------- planning

        PlannerContext Context()
        {
            return new PlannerContext
            {
                Stage = m_Stage,
                SimTime = m_SimTime,
                MosquitoPos = transform.position,
                Landed = m_Motor.IsLanded,
                FeedProgress = m_FeedTimer,
                FeedTime = m_P.FeedTime,
                Human = m_Area.human,
                Hand = m_Area.hand,
                Candidates = m_Area.Candidates.Current,
                ExcludedSkins = m_Excluded,
                Current = m_Tracker.Command,
                HideSituation = m_HideScenario && !m_Chosen,
            };
        }

        void Decide(bool decisionStep)
        {
            if ((m_Probe || m_HideScenario) && m_DecisionIndex > 0) { m_Events.Clear(); return; } // one decision only
            bool feeding = m_Motor.IsLanded && m_Tracker.Command.Subgoal == Subgoal.Approach;
            bool random = m_P.Planner == PlannerKind.Random;

            // Training: random mid-switch, about once per 5 s, never while feeding or holding a hiding spot
            // (the 5 s hold would rarely finish otherwise). Events can still switch.
            bool midSwitch = random && decisionStep && !feeding && !m_Tracker.Holding && m_Stage >= 1 &&
                             m_Rng.NextDouble() < k_MidSwitchProb;

            if (random && feeding)
            {
                // Feeding lock: only threat events (stage 2+) may switch while landed (final review 1-2).
                bool threat = m_Events.Contains("alert_start") || m_Events.Contains("hand_active");
                if (!threat) m_Events.Clear();
            }

            if (m_Events.Count == 0 && !midSwitch) return;
            if (!midSwitch && m_SimTime - m_LastDecisionTime < k_MinDecisionInterval) return; // keep events pending

            if (m_Stage >= 1 || m_P.Planner == PlannerKind.External)
            {
                var rng = new System.Random(EpisodeSeed.Derive(m_Seed, m_DecisionIndex));
                m_Area.Candidates.Select(rng, transform.position, m_Area.human, m_Area.net, m_Area.Cues);
            }
            var snapshot = new List<Candidate>(m_Area.Candidates.Current);
            m_Snapshots[m_DecisionIndex] = snapshot;
            PlannerContext ctx = Context();

            switch (m_P.Planner)
            {
                case PlannerKind.Fixed:
                    Apply(Planners.Fixed(ctx), snapshot, ctx);
                    break;
                case PlannerKind.Rule:
                    Apply(Planners.Rule(ctx), snapshot, ctx);
                    break;
                case PlannerKind.Random:
                    Apply(Planners.Random(ctx, m_Rng), snapshot, ctx);
                    break;
                case PlannerKind.External:
                    SendRequest(ctx);
                    break;
                case PlannerKind.Probe:
                    Apply(Planners.Probe(ctx, m_Rng, m_ProbeSubgoal), snapshot, ctx); // HIDE: SetCommand starts the search
                    if (!m_HideScenario) StartProbe(snapshot);
                    break;
            }

            m_LastDecisionTime = m_SimTime;
            m_DecisionIndex++;
            m_Events.Clear();
        }

        void Apply(PlannerCommand cmd, List<Candidate> snapshot, PlannerContext ctx)
        {
            SetCommand(cmd, snapshot);
            if (PlannerChannel.Instance != null && PlannerChannel.Instance.Log)
            {
                var j = new JsonBuilder();
                j.BeginObject().Key("type").Value("decision");
                WriteDecisionFields(j, ctx);
                j.Key("command"); cmd.WriteJson(j);
                j.EndObject();
                PlannerChannel.Instance.Send(j.ToString());
            }
        }

        void SendRequest(PlannerContext ctx)
        {
            var ch = PlannerChannel.Instance;
            if (ch == null || !ch.Configured) return; // nobody is listening: keep the current command

            var j = new JsonBuilder();
            j.BeginObject().Key("type").Value("request");
            WriteDecisionFields(j, ctx);
            // The masked rule planner (Python) reads only "view"; the instruction is rendered from it.
            PlannerView view = StateDescriber.View(ctx, episodeSeconds);
            j.Key("view"); view.WriteJson(j);
            j.Key("instruction").Value(StateDescriber.Describe(view));

            if (ch.CaptureImages && m_Area.cameras != null)
            {
                Vector3 look = m_TargetSkin != null ? m_TargetSkin.Center : m_Area.human.Chest;
                string dir = System.IO.Path.Combine(ch.ImageDir, m_EpisodeId.Replace('/', '_'));
                string[] paths = m_Area.cameras.Capture(dir, "d" + m_DecisionIndex.ToString("000"), ch.ImageSize,
                    transform.position, look, ctx.Candidates, m_Area.Skins);
                j.Key("images").BeginArray();
                foreach (var p in paths) j.Value(p);
                j.EndArray();
            }

            j.Key("rule_command"); Planners.Rule(ctx).WriteJson(j); // oracle rule: knows hidden / clear_path
            j.Key("fixed_command"); Planners.Fixed(ctx).WriteJson(j);
            j.EndObject();
            ch.Send(j.ToString());
        }

        void WriteDecisionFields(JsonBuilder j, PlannerContext ctx)
        {
            j.Key("room").Value(m_Area.roomIndex);
            j.Key("episode").Value(m_EpisodeId);
            j.Key("decision").Value(m_DecisionIndex);
            j.Key("sim_time").Value(m_SimTime);
            j.Key("stage").Value(m_Stage);
            j.Key("events").BeginArray();
            foreach (var e in m_Events) j.Value(e);
            j.EndArray();
            j.Key("current_command"); ctx.Current.WriteJson(j);
            j.Key("landed").Value(ctx.Landed);
            j.Key("alert").Value(ctx.AlertKnown);
            j.Key("hand_active").Value(m_Stage >= 2 && ctx.Hand.Active);
            j.Key("hand_dist").Value(Vector3.Distance(ctx.Hand.Position, ctx.MosquitoPos));

            j.Key("skins").BeginArray();
            foreach (var s in ctx.Human.skins)
            {
                j.BeginObject()
                    .Key("id").Value(s.id)
                    .Key("name").Value(s.zoneName)
                    .Key("dist").Value(Vector3.Distance(s.Center, ctx.MosquitoPos))
                    .Key("hand_dist").Value(Vector3.Distance(s.Center, ctx.Hand.Position))
                    .Key("covered").Value(s.covered)
                    .Key("excluded").Value(m_Excluded.Contains(s.id))
                    .EndObject();
            }
            j.EndArray();

            j.Key("candidates").BeginArray();
            foreach (var c in ctx.Candidates)
            {
                j.BeginObject()
                    .Key("id").Value(c.Id)
                    .Key("pos").Value(c.Pos)
                    .Key("dist").Value(c.Dist)
                    .Key("hidden").Value(c.Hidden)
                    .Key("clear_path").Value(c.ClearPath)
                    .Key("near_trap").Value(c.NearTrap)
                    .EndObject();
            }
            j.EndArray();
        }

        void PollExternal()
        {
            var ch = PlannerChannel.Instance;
            if (ch == null) return;
            while (ch.TryDequeue(m_Area.roomIndex, out PlannerInMessage m))
            {
                if (m.episode != m_EpisodeId) continue; // answer to a previous episode
                if (m_HideScenario && m_Chosen) continue; // one choice per HIDE scenario
                if (!PlannerCommand.TryFromDto(m.command, out PlannerCommand cmd)) continue;
                m_Snapshots.TryGetValue(m.decision, out List<Candidate> snapshot);
                if (!IsValidTarget(cmd, snapshot)) continue;
                SetCommand(cmd, snapshot);
            }
        }

        bool IsValidTarget(PlannerCommand cmd, List<Candidate> snapshot)
        {
            if (m_HideScenario && cmd.Subgoal != Subgoal.Hide) return false; // only a hiding spot is asked for
            switch (cmd.Subgoal)
            {
                case Subgoal.Approach:
                    return m_Area.human.SkinById(cmd.Target) != null;
                case Subgoal.Hide:
                case Subgoal.MoveTo:
                    if (snapshot == null) return false;
                    foreach (var c in snapshot) if (c.Id == cmd.Target) return true;
                    return false;
                default:
                    return true;
            }
        }

        void SetCommand(PlannerCommand cmd, List<Candidate> snapshot)
        {
            if (cmd.Subgoal == Subgoal.Approach)
            {
                var skin = m_Area.human.SkinById(cmd.Target);
                if (skin != null) m_TargetSkin = skin;
            }
            m_Tracker.Set(cmd, ResolveTarget(cmd, snapshot), m_SimTime, transform.position);
            m_Guide.Reset();
            if (Guided)
            {
                m_Guide.Update(m_Area.Nav, transform.position, m_Tracker.Target, m_SimTime);
                m_Tracker.Shaping(m_Guide.Remaining); // start shaping from the route length
            }
            if (m_HideScenario && !m_Chosen && cmd.Subgoal == Subgoal.Hide) ChooseHide(snapshot);
        }

        Vector3 ResolveTarget(PlannerCommand cmd, List<Candidate> snapshot)
        {
            switch (cmd.Subgoal)
            {
                case Subgoal.Approach:
                    return m_TargetSkin != null ? m_TargetSkin.Center : m_Area.human.Chest;
                case Subgoal.Evade:
                    return m_Area.hand.Position;
                default:
                    if (snapshot != null)
                        foreach (var c in snapshot)
                            if (c.Id == cmd.Target) return c.Pos;
                    return m_Tracker.Target; // keep the point chosen when the command was set
            }
        }

        // ---------------------------------------------------------------- bookkeeping

        void RecordPreviousEpisode()
        {
            if (!m_HasEpisode) return;
            if ((m_Probe || m_HideScenario) && !m_ProbeDone)
                FinishProbe(m_Caught ? "caught" : m_Trapped ? "trapped" : "timeout");
            Episodes++;
            if (m_Succeeded) Successes++;
            if (m_Caught) Caught++;
            if (m_Trapped) Trapped++;
            Decisions += m_DecisionIndex;
            var stats = Academy.Instance.StatsRecorder;
            stats.Add("Mosquito/Success", m_Succeeded ? 1f : 0f);
            stats.Add("Mosquito/Caught", m_Caught ? 1f : 0f);
            stats.Add("Mosquito/Trapped", m_Trapped ? 1f : 0f);
            stats.Add("Mosquito/Collisions", m_Collisions);

            var ch = PlannerChannel.Instance;
            if (ch != null && ch.Configured)
            {
                var j = new JsonBuilder();
                j.BeginObject()
                    .Key("type").Value("episode_end")
                    .Key("room").Value(m_Area.roomIndex)
                    .Key("episode").Value(m_EpisodeId)
                    .Key("stage").Value(m_Stage)
                    .Key("success").Value(m_Succeeded)
                    .Key("caught").Value(m_Caught)
                    .Key("trapped").Value(m_Trapped)
                    .Key("timeout").Value(!m_Succeeded && !m_Caught && !m_Trapped)
                    .Key("sim_time").Value(m_SimTime)
                    .Key("collisions").Value(m_Collisions)
                    .Key("decisions").Value(m_DecisionIndex)
                    .Key("reward").Value(GetCumulativeReward())
                    .Key("room_type").Value(m_Area.Generator.Current.ToString())
                    .Key("layout").Value(m_Area.Obstacles.Layout);
                if (m_Probe || m_HideScenario)
                {
                    j.Key("probe_subgoal").Value(PlannerCommand.WireName(m_ProbeSubgoal))
                        .Key("probe_target").Value(m_ProbeTarget)
                        .Key("probe_outcome").Value(m_ProbeOutcome)
                        .Key("probe_time").Value(m_ProbeTime)
                        .Key("probe_start_dist").Value(m_ProbeStartDist)
                        .Key("probe_min_dist").Value(m_ProbeMinDist)
                        .Key("probe_path_blocked").Value(m_ProbePathBlocked)
                        .Key("probe_target_height").Value(m_ProbeTargetHeight)
                        .Key("probe_target_hidden").Value(m_ProbeTargetHidden)
                        .Key("probe_spotted").Value((m_Stage >= 2 || m_HideScenario) && m_Area.human.SpottedEver)
                        .Key("probe_valid").Value(!m_HideScenario || m_ProbeValid)
                        .Key("probe_wait").Value(m_ProbeWait)
                        .Key("probe_travel").Value(m_ProbeTravel)
                        .Key("probe_arrived").Value(m_ProbeArrived);
                }
                j.EndObject();
                ch.Send(j.ToString());
            }
        }

        // ---------------------------------------------------------------- direct subgoal check

        /// <summary>
        /// Planner = Probe: choose this episode's subgoal. HIDE (probe, or any planner with
        /// hide_scenario): the HIDE scenario. The mosquito starts 0.5-1.0 m from the head,
        /// in the human's sight; mosquito and human hold still (the human already alert)
        /// until a HIDE choice arrives. EVADE: the mosquito starts near the hand, which
        /// swings after a fixed delay. The other subgoals start like any episode.
        /// </summary>
        void SetUpProbe()
        {
            m_Probe = m_P.Planner == PlannerKind.Probe;
            m_ProbeStarted = m_ProbeDone = false;
            m_ProbeOutcome = null;
            m_ProbeTarget = null;
            m_ProbeTime = m_ProbeHold = m_ProbeWait = m_ProbeTravel = 0f;
            m_Chosen = m_Searching = m_ProbeValid = m_ProbeArrived = false;

            if (m_Probe)
            {
                var allowed = Planners.StageSubgoals(m_Stage);
                m_ProbeSubgoal = m_P.ProbeSubgoal >= 0 ? (Subgoal)m_P.ProbeSubgoal : allowed[m_EpisodeIndex % allowed.Count];
                if (!allowed.Contains(m_ProbeSubgoal))
                {
                    Debug.LogWarning($"probe: {m_ProbeSubgoal} is not used in stage {m_Stage}; checking APPROACH");
                    m_ProbeSubgoal = Subgoal.Approach;
                }
            }
            m_HideScenario = m_Stage >= 1 && ((m_Probe && m_ProbeSubgoal == Subgoal.Hide) ||
                                              (m_P.Planner == PlannerKind.External && m_P.HideScenario));

            var human = m_Area.human;
            if (m_HideScenario)
            {
                m_ProbeSubgoal = Subgoal.Hide;
                if (TryPlaceNear(human.HeadPos, k_HideStartMin, k_HideStartMax, float.MaxValue, out Vector3 p)) m_Motor.ResetMotor(p);
                m_SoundPos = transform.position;
                m_WaitStart = 0f; // the episode clock restarts right after this
                human.PrepareAlert();
            }
            else if (m_Probe && m_ProbeSubgoal == Subgoal.Evade)
            {
                var hand = m_Area.hand;
                if (TryPlaceNear(hand.Position, k_ProbeHandMin, k_ProbeHandMax, HandController.Reach, out Vector3 p)) m_Motor.ResetMotor(p);
                hand.ForceSwing(k_ProbeSwingDelay);
            }
        }

        /// <summary>HIDE scenario: the choice is in (or the wait ran out). The mosquito flies; the human stays still.</summary>
        void ChooseHide(List<Candidate> snapshot)
        {
            m_Chosen = true;
            m_ProbeValid = snapshot != null;
            m_ProbeWait = m_SimTime - m_WaitStart;
            m_TravelStart = m_SimTime;
            if (m_ProbeValid) StartProbe(snapshot);
            else { m_ProbeStarted = true; m_ProbeStartDist = m_ProbeMinDist = 0f; }
        }

        /// <summary>HIDE scenario: the mosquito reached its spot (or ran out of time); the human turns and looks.</summary>
        void BeginHideSearch(bool arrived)
        {
            m_Searching = true;
            m_ProbeArrived = arrived;
            m_ProbeTravel = m_SimTime - m_TravelStart;
            m_SearchStart = m_SimTime;
            var h = m_Area.human;
            Vector3 toM = transform.position - h.HeadPos; toM.y = 0f;
            Vector3 fwd = h.transform.forward; fwd.y = 0f;
            m_SearchAngle = Vector3.Angle(fwd, toM);
            h.BeginSearch(m_SoundPos);
        }

        /// <summary>A free point <paramref name="min"/>-<paramref name="max"/> m from <paramref name="center"/>,
        /// within <paramref name="shoulderReach"/> of the human's shoulder height, in the human's sight.</summary>
        bool TryPlaceNear(Vector3 center, float min, float max, float shoulderReach, out Vector3 p)
        {
            var human = m_Area.human;
            Vector3 shoulder = human.transform.TransformPoint(new Vector3(0.22f, 1.35f, 0f));
            for (int i = 0; i < 60; i++)
            {
                Vector3 dir = new Vector3((float)m_Rng.NextDouble() * 2f - 1f, (float)m_Rng.NextDouble() * 1.2f - 0.6f,
                    (float)m_Rng.NextDouble() * 2f - 1f);
                if (dir.sqrMagnitude < 1e-3f) continue;
                p = center + dir.normalized * (min + (float)m_Rng.NextDouble() * (max - min));
                if (Vector3.Distance(p, shoulder) > shoulderReach) continue;
                if (Physics.CheckSphere(p, 0.05f, ~(1 << Layers.Labels), QueryTriggerInteraction.Ignore)) continue;
                if (!human.CanSee(p)) continue;
                return true;
            }
            p = transform.position;
            return false;
        }

        void StartProbe(List<Candidate> snapshot)
        {
            Vector3 pos = transform.position;
            var cmd = m_Tracker.Command;
            m_ProbeStarted = true;
            m_ProbeTarget = cmd.Target;
            m_ProbeStartDist = m_ProbeMinDist = Vector3.Distance(pos, m_Tracker.Target);
            m_ProbePathBlocked = cmd.Subgoal != Subgoal.Evade &&
                Physics.Linecast(pos, m_Tracker.Target, Layers.WorldNoHuman, QueryTriggerInteraction.Ignore);
            m_ProbeTargetHeight = m_Tracker.Target.y - m_Area.human.transform.position.y;
            m_ProbeTargetHidden = false;
            if (snapshot != null)
                foreach (var c in snapshot)
                    if (c.Id == cmd.Target) m_ProbeTargetHidden = c.Hidden;
        }

        /// <summary>
        /// Success rules (11/22): APPROACH lands on the target skin (another skin fails);
        /// MOVE_TO reaches 0.15 m; (HIDE is the scenario: see SetUpProbe); EVADE is not caught until the swing ends or it is
        /// 1.5 m from the hand. All within 8 s. Returns true when the probe has ended.
        /// </summary>
        bool JudgeProbe(float dt, Vector3 pos, SubgoalStatus status)
        {
            float d = Vector3.Distance(pos, m_Tracker.Target);
            m_ProbeMinDist = Mathf.Min(m_ProbeMinDist, d);
            string outcome = null;
            switch (m_ProbeSubgoal)
            {
                case Subgoal.Approach:
                    if (m_Motor.IsLanded && m_Motor.LandedZone != null)
                        outcome = m_Motor.LandedZone.id == m_ProbeTarget ? "achieved" : "wrong_skin";
                    break;
                default: // MOVE_TO, EVADE: the training rules
                    if (status == SubgoalStatus.Achieved) outcome = "achieved";
                    break;
            }
            if (outcome == null && m_SimTime - m_Tracker.StartTime > SubgoalConstants.TimeoutSeconds) outcome = "timeout";
            if (outcome == null) return false;
            FinishProbe(outcome);
            return true;
        }

        void FinishProbe(string outcome)
        {
            m_ProbeDone = true;
            m_ProbeOutcome = outcome;
            m_ProbeTime = m_SimTime - (m_HideScenario ? m_SearchStart : m_Tracker.StartTime);
            string n = PlannerCommand.WireName(m_ProbeSubgoal);
            ProbeStats.TryGetValue(n, out Vector3 s);
            s.x += 1f;
            if (outcome == "achieved") { s.y += 1f; s.z += m_ProbeTime; }
            ProbeStats[n] = s;
            ProbeOutcomes.TryGetValue(n + "/" + outcome, out Vector2 o);
            ProbeOutcomes[n + "/" + outcome] = new Vector2(o.x + 1f, o.y + m_ProbeTime);
            if (m_HideScenario && HideTrace.Count < 12)
                HideTrace.Add($"{outcome} {m_ProbeTime:0.00}s  도착 {(m_ProbeArrived ? "O" : "X")} 이동시간 {m_ProbeTravel:0.0}s  시작 시 몸 앞과 모기 각 {m_SearchAngle:0}°  " +
                              $"목표까지 {m_ProbeStartDist:0.00}m → 최소 {m_ProbeMinDist:0.00}m  목표 가려짐 {(m_ProbeTargetHidden ? "O" : "X")}  " +
                              $"이동 {Vector3.Distance(transform.position, m_SoundPos):0.00}m");
            var stats = Academy.Instance.StatsRecorder;
            stats.Add("Probe/" + n + "/Achieved", outcome == "achieved" ? 1f : 0f);
        }

        /// <summary>
        /// MOVE_TO / HIDE: route around obstacles (NavGrid A*) and shape on the
        /// route length, so a target behind furniture or the net is not a dead
        /// end for a policy that only sees a straight-line direction. Other
        /// subgoals shape on the straight distance as before. Returns the
        /// shaping progress, clamped to what the mosquito can fly in one step
        /// (a replan can shorten the route abruptly).
        /// </summary>
        float UpdateGuide(Vector3 pos)
        {
            if (!Guided)
            {
                if (m_Guide.Active) m_Guide.Reset();
                return m_Tracker.Shaping(pos);
            }
            m_Guide.Update(m_Area.Nav, pos, m_Tracker.Target, m_SimTime);
            GuidedSteps++;
            if (!m_Guide.Active) UnroutedSteps++;
            float step = ObsLayout.MaxSpeed * Time.fixedDeltaTime * 1.5f;
            return Mathf.Clamp(m_Tracker.Shaping(m_Guide.Remaining), -step, step);
        }

        /// <summary>
        /// Scripted flight for auto tests: P-control toward the subgoal target,
        /// slowing to landing speed near it; EVADE flies straight away. No
        /// obstacle avoidance, so it is only meant for checking the simulator.
        /// </summary>
        Vector3 Autopilot()
        {
            Vector3 pos = transform.position;
            Vector3 to = GuideTarget - pos;
            float dist = to.magnitude;
            Vector3 dir = dist > 1e-4f ? to / dist : Vector3.zero;
            Vector3 desired;
            if (m_Tracker.Command.Subgoal == Subgoal.Evade)
                desired = -dir * 1.2f;
            else if (m_Motor.IsLanded && m_Tracker.Command.Subgoal != Subgoal.Approach)
                desired = m_Motor.LandedNormal * 1.0f; // take off
            else
                desired = dir * Mathf.Clamp(dist * 1.2f, 0.1f, 1.2f);
            Vector3 dv = desired - m_Motor.Velocity;
            return Vector3.ClampMagnitude(dv / 0.5f, 1f);
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            // Keyboard control for checking a scene: WASD move, Space/Shift up/down.
            var c = actionsOut.ContinuousActions;
            var view = MosquitoView.Instance;
            // In the free camera WASD moves the camera, so nobody flies by keyboard then.
            bool watched = view != null && m_Area != null && view.roomIndex == m_Area.roomIndex && view.mode != MosquitoView.Mode.Free;
            bool autopilot = (AutoTestRunner.Active && AutoTestRunner.F("autopilot", 0f) > 0.5f) ||
                             (SimSettings.ManualPlay && (SimSettings.Instance.flight == SimSettings.Flight.Autopilot || !watched));
            if (autopilot)
            {
                Vector3 ap = Autopilot();
                c[0] = ap.x;
                c[1] = ap.y;
                c[2] = ap.z;
                return;
            }
            float x = 0f, y = 0f, z = 0f;
#if ENABLE_INPUT_SYSTEM
            var k = Keyboard.current;
            if (k != null)
            {
                x = (k.dKey.isPressed ? 1f : 0f) - (k.aKey.isPressed ? 1f : 0f);
                z = (k.wKey.isPressed ? 1f : 0f) - (k.sKey.isPressed ? 1f : 0f);
                y = (k.spaceKey.isPressed ? 1f : 0f) - (k.leftShiftKey.isPressed ? 1f : 0f);
            }
#elif ENABLE_LEGACY_INPUT_MANAGER
            x = Input.GetAxisRaw("Horizontal");
            z = Input.GetAxisRaw("Vertical");
            y = (Input.GetKey(KeyCode.Space) ? 1f : 0f) - (Input.GetKey(KeyCode.LeftShift) ? 1f : 0f);
#endif
            // Relative to the camera: W flies where the view looks (level), D to its right.
            if (view != null)
            {
                Vector3 f = view.transform.forward;
                f.y = 0f;
                if (f.sqrMagnitude > 1e-6f)
                {
                    f.Normalize();
                    Vector3 r = new Vector3(f.z, 0f, -f.x);
                    Vector3 w = r * x + f * z;
                    x = w.x;
                    z = w.z;
                }
            }
            c[0] = x;
            c[1] = y;
            c[2] = z;
        }
    }
}
