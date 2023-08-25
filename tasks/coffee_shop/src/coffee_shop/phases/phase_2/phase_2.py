import smach
from .states import TakeOrder, MakeOrder, CheckOrder, GoToTable, DeliverOrder, GoToCounter, InvalidateOrder, HandOverOrder, Start

class Phase2(smach.StateMachine):
    def __init__(self, base_controller, head_controller, voice_controller, yolo, tf, pm, speech):
        smach.StateMachine.__init__(self, outcomes=['done'])
    
        with self:
            smach.StateMachine.add('START_PHASE_2', Start(voice_controller), transitions={'done' : 'GO_TO_TABLE'})
            smach.StateMachine.add('GO_TO_TABLE', GoToTable(base_controller, voice_controller), transitions={'done' : 'TAKE_ORDER', 'skip' : 'done'})
            smach.StateMachine.add('TAKE_ORDER', TakeOrder(head_controller, voice_controller, speech), transitions={'done' : 'GO_TO_COUNTER'})
            smach.StateMachine.add('GO_TO_COUNTER', GoToCounter(base_controller, voice_controller), transitions={'done' : 'MAKE_ORDER'})
            smach.StateMachine.add('MAKE_ORDER', MakeOrder(voice_controller), transitions={'done' : 'CHECK_ORDER'})
            smach.StateMachine.add('CHECK_ORDER', CheckOrder(yolo, tf, pm), transitions={'correct' : 'DELIVER_ORDER', 'incorrect' : 'INVALIDATE_ORDER'})
            smach.StateMachine.add('INVALIDATE_ORDER', InvalidateOrder(voice_controller), transitions={'done' : 'CHECK_ORDER'})
            smach.StateMachine.add('DELIVER_ORDER', DeliverOrder(base_controller, voice_controller), transitions={'done' : 'HAND_OVER_ORDER'})
            smach.StateMachine.add('HAND_OVER_ORDER', HandOverOrder(voice_controller), transitions={'done' : 'done'})
