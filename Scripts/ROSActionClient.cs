using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Actionlib;
using RosSharp.RosBridgeClient.MessageTypes.Action;

namespace IPC
{
    /// <summary> May God have mercy on your soul if you dare try to touch this.
    /// https://github.com/siemens/ros-sharp/wiki/User_App_DotNet_Fibonacci_Action#-441-fibonacci-client
    /// <para> Pray. Pray alot. </para>
    /// <para> Look at ROS.AdvertiseAction for more info </para> </summary>
    public class ROSActionClient<A, B, C, D, E, F, G> :
    ActionClient<A, B, C, D, E, F, G>
    where A : Action<B, C, D, E, F, G>
    where B : ActionGoal<E> where C : ActionResult<F> where D : ActionFeedback<G>
    where E : Message where F : Message where G : Message
    {
        // Lambda funcs
        private System.Action FeedbackCallback, ResultCallback, StatusCallback;

        public ROSActionClient(
            string _ActionName, A _Action,
            SendActionGoalHandler<B> _SendActionGoal, CancelActionGoalHandler _CancelActionGoal,
            GoalStatus _GoalStatus,
            System.Action _FeedbackCallback, System.Action _ResultCallback, System.Action _StatusCallback)
        {
            actionName = _ActionName;
            rosSocket = ROS.ROSSocket;

            action = _Action;
            goalStatus = _GoalStatus != null ? _GoalStatus : new GoalStatus();

            FeedbackCallback = _FeedbackCallback != null ? _FeedbackCallback : () => { };
            ResultCallback = _ResultCallback != null ? _ResultCallback : () => { };
            StatusCallback = _StatusCallback != null ? _StatusCallback : () => { };

            action.action_goal.action = _ActionName;
            action.action_feedback.action = _ActionName;
            action.action_result.action = _ActionName;

            // Not necessary to changes these at this time
            action.action_goal.fragment_size = int.MaxValue;
            action.action_goal.feedback = true;
            action.action_goal.compression = "none";

            // The actual action advertisement happens here, would honestly be strange to put this
            // in ROS.cs instead of here (though it probably could be)
            ROS.ROSSocket.AdvertiseAction<B, C, D>(_ActionName, _SendActionGoal, _CancelActionGoal);
        }

        /// <summary> Even though action is public </summary>
        public override B GetActionGoal() => action.action_goal;

        public void PublishActionGoal(B actionGoal)
        {
            action.action_goal = actionGoal;
            SendGoal();
        }

        public void PublishActionGoal(E Goal)
        {
            action.action_goal.args = Goal;
            SendGoal();
        }

        /// <summary> Prefer PublishActionGoal </summary>
        public override void SetActionGoal(E args, bool feedback = true, int fragmentSize = int.MaxValue, string compression = "none")
        {
            action.action_goal.args = args;
            action.action_goal.feedback = feedback;
            action.action_goal.fragment_size = fragmentSize;
            action.action_goal.compression = compression;
        }

        // Literally no reason to touch these.
        // unless... custom global callback action?
        protected override void OnFeedbackReceived() => FeedbackCallback.Invoke();
        protected override void OnResultReceived() => ResultCallback.Invoke();
        protected override void OnStatusUpdated() => StatusCallback.Invoke();
    }
}
