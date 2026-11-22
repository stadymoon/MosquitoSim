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

        public PlannerCommand CurrentCommand => m_Tracker.Command;
        public float SimTime => m_SimTime;
        public Vector3 SubgoalTarget => m_Tracker.Target;
        public float FeedTimer => m_FeedTimer;
        public bool Landed => m_Motor != null && m_Motor.IsLanded;
        public Vector3 Velocity => m_Motor != null ? m_Motor.Velocity : Vector3.zero;
        public int Stage => m_Stage;
        public RoomArea Area => m_Area;
        public float[] LastObservation => m_Obs;

        /// <summary>Episode outcome counts across all agents (read by AutoTestRunner).</summary>
        public static int Episodes, Successes, Caught, Trapped, Decisions;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void ResetCounters() { Episodes = Successes = Caught = Trapped = Decisions = 0; }

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
                TargetSkinRel = (m_TargetSkin != null ? m_TargetSkin.Center : m_Tracker.Target) - pos,
                SubgoalTargetRel = m_Tracker.Target - pos,
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

            var cmd = m_Tracker.Command;
            SkinZone landing = cmd.Subgoal == Subgoal.Approach && m_TargetSkin != null && !m_TargetSkin.covered
                ? m_TargetSkin : null;
            MotorStepResult r = m_Motor.Step(action, dt, landing, m_P.LandingRadius, m_P.LandingSpeed);
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

            // Subgoal shaping and end
            m_Tracker.UpdateTarget(ResolveTarget(m_Tracker.Command, null));
            if (!m_Motor.IsLanded) AddReward(k_ProgressCoef * m_Tracker.Shaping(pos));
            else m_Tracker.Shaping(pos);
            bool hazard = m_Stage >= 2 && hand.Active;
            SubgoalStatus status = m_Tracker.Step(m_SimTime, dt, pos, m_Motor.IsLanded, hazard);
            if (status == SubgoalStatus.Achieved)
            {
                if (m_Tracker.Command.Subgoal != Subgoal.Approach) AddReward(k_AchieveBonus);
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
            if (m_Motor.IsLanded && m_Motor.LandedZone != null && !m_Motor.LandedZone.covered)
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
            };
        }

        void Decide(bool decisionStep)
        {
            bool feeding = m_Motor.IsLanded && m_Tracker.Command.Subgoal == Subgoal.Approach;
            bool random = m_P.Planner == PlannerKind.Random;

            // Training: random mid-switch, about once per 5 s, never while feeding.
            bool midSwitch = random && decisionStep && !feeding && m_Stage >= 1 && m_Rng.NextDouble() < k_MidSwitchProb;

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
            j.Key("instruction").Value(StateDescriber.Describe(ctx, episodeSeconds));

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

            j.Key("rule_command"); Planners.Rule(ctx).WriteJson(j);
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
            j.Key("alert").Value(m_Stage >= 2 && ctx.Human.Alert);
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
                if (!PlannerCommand.TryFromDto(m.command, out PlannerCommand cmd)) continue;
                m_Snapshots.TryGetValue(m.decision, out List<Candidate> snapshot);
                if (!IsValidTarget(cmd, snapshot)) continue;
                SetCommand(cmd, snapshot);
            }
        }

        bool IsValidTarget(PlannerCommand cmd, List<Candidate> snapshot)
        {
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
                    .Key("room_type").Value(m_Area.Generator.Current.ToString())
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
                    .EndObject();
                ch.Send(j.ToString());
            }
        }

        /// <summary>
        /// Scripted flight for auto tests: P-control toward the subgoal target,
        /// slowing to landing speed near it; EVADE flies straight away. No
        /// obstacle avoidance, so it is only meant for checking the simulator.
        /// </summary>
        Vector3 Autopilot()
        {
            Vector3 pos = transform.position;
            Vector3 to = m_Tracker.Target - pos;
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
