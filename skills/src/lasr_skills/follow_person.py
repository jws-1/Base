#!/usr/bin/env python3

import rospy
import smach


class FollowPerson(smach.StateMachine):

    class Begin(smach.StateMachine):

        def __init__(self):
            smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

            """
            Begin by waiting for a person to be detected, looking at them, and then asking if they want to be followed.
            """

    class GetPose(smach.StateMachine):
        pass

    class GoToPose(smach.StateMachine):
        pass

    class CheckFinished(smach.StateMachine):
        """
        Will check if the person has been static for a certain amount of time, and also update the pose queue to remove old poses.
        """

        pass

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "BEGIN",
                FollowPerson.Begin(),
                transitions={"succeeded": "FOLLOW", "failed": "failed"},
            )

            """
            Concurrently get the pose of the person and move the robot to follow them.
            """

            sm_con = smach.Concurrence(
                outcomes=["succeeded", "failed"],
                default_outcome="failed",
                outcome_map={
                    "succeeded": {
                        "GET_POSE": "succeeded",
                        "GO_TO_POSE": "succeeded",
                    }
                },
                input_keys=["pose_queue"],
                output_keys=["pose_queue"],
            )

            with sm_con:
                smach.Concurrence.add("GET_POSE", FollowPerson.GetPose())
                smach.Concurrence.add("GO_TO_POSE", FollowPerson.GoToPose())

            smach.StateMachine.add(
                "FOLLOW", sm_con, transitions={"succeeded": "CHECK_FINISHED"}
            )
            smach.StateMachine.add(
                "CHECK_FINISHED",
                FollowPerson.CheckFinished(),
                transitions={"succeeded": "succeeded", "failed": "FOLLOW"},
            )
