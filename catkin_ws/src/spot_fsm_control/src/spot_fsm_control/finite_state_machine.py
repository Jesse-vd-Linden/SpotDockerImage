#!/usr/bin/env python


from statemachine import StateMachine, State
# from spot_control_interface import SpotControlInterface

class SpotStateMachine(StateMachine):
    """
    A state machine for Spot to control all the inputs from the NUI to a command to the robot
    The state machine is used to increase the safety of operation and prevent initiation of states that cannot match with other states.
    """
    sit = State(initial=True)

    # Stand
    stand = State()
    stand_high = State()
    stand_low = State()
    sit_down = ( stand.to(sit) | stand_high.to(sit) | stand_low.to(sit) )
    stand_up = ( sit.to(stand) | stand_high.to(stand) | stand_low.to(stand) )
    stand_up_high = (stand.to(stand_high) | sit.to(stand_high)|stand_low.to(stand_high))
    stand_up_low = (stand.to(stand_low) | sit.to(stand_low) | stand_high.to(stand_low))

    walk_forward = State()
    walk_backward = State()
    walk_left = State()
    walk_right = State()

    walk_to_forward = stand.to(walk_forward)
    walk_to_backward = stand.to(walk_backward)
    walk_to_left = stand.to(walk_left)
    walk_to_right = stand.to(walk_right)

    deitic_location_movement = State()
    move_to_location = stand.to(deitic_location_movement)

    face_operator = State()
    face_to_operator = stand.to(face_operator)
    
    pick_object = State()
    pick_up_object = stand.to(pick_object)


    turn_left = State()
    turn_right = State()
    turn_to_left = stand.to(turn_left)
    turn_to_right = stand.to(turn_right)
    
    

    gaze_control = State()
    arm_trajectory = State()
    start_gaze = stand.to(gaze_control)
    start_trajectory = stand.to(arm_trajectory)
    
    stop_action = (
        walk_forward.to(stand) |
        walk_backward.to(stand) |
        walk_left.to(stand) |
        walk_right.to(stand) |
        turn_right.to(stand) |
        turn_left.to(stand) |
        deitic_location_movement.to(stand) |
        face_operator.to(stand) |
        pick_object.to(stand) |
        arm_trajectory.to(stand) |
        gaze_control.to(stand)
    )
    
    turn_off = State(final=True)
    turn_off_robot = sit.to(turn_off)

    def __init__(self, robot):
        self.robot = robot
        super().__init__()
        
    def after_stop_action(self):
        print("Action stopped.")
        
    def on_enter_walk_forward(self):
        self.robot.forward = 0.4
        self.robot.move_command(duration=2)
        print("move forward")

    def on_enter_walk_backward(self):
        self.robot.forward = -0.4
        self.robot.move_command(duration=2)
        print("move backward")

    def on_enter_stop_walk(self):
        self.robot.stop()
        print("Stop")

    def on_enter_turn_left(self):
        self.robot.rotate = 0.2
        self.robot.move_command(duration=2)
        print("Rotate left")

    def on_enter_turn_right(self):
        self.robot.rotate = -0.2
        self.robot.move_command(duration=2)
        print("Rotate right")

    def on_enter_walk_left(self):
        self.robot.strafe = 0.2
        self.robot.move_command(duration=2)
        print("Move left")

    def on_enter_walk_right(self):
        self.robot.strafe = -0.2
        self.robot.move_command(duration=2)
        print("Move right")

    def on_exit_arm_trajectory(self):
        self.robot.stand(0.0)

    def on_enter_stand(self):
        if self.robot:
            self.robot.stand(0.0)
            print(f"Standing")
    
    def on_enter_stand_high(self):
        self.robot.stand(0.1)
        print(f"Standing high")

    def on_enter_stand_low(self):
        self.robot.stand(-0.1)
        print(f"Standing low")

    def on_enter_sit(self):
        if self.robot:
            self.robot.sit_down()
            print(f"Sit down.")

    def on_enter_gaze_control(self):
        print("Gaze Control.")
        self.robot.gaze_control()

    def on_enter_arm_trajectory(self):
        self.robot.arm_trajectory()

    def on_enter_turn_off(self):
        self.robot.stand(0.0)
        self.robot.sit_down()


if __name__ == "__main__":
    spot = SpotStateMachine(robot=None)#SpotControlInterface())

    img_path = "docs/images/readme_spotstatemachine1.png"
    spot._graph().write_png(img_path)
    
    msg = spot.send("stand_up")
    print(msg)

    spot.send("stand_up_high")
    
    ## How to error handle wrong actions to state machine
    try:
        spot.send("start_trajectory")
    except:
        try:
            spot.send("stop_walking")
            spot.send("start_trajectory")
        except:
            try:
                spot.send("stand_up")
                print("Stand up first")
                spot.send("start_trajectory")
            except:
                print("Start trajectory not possible")
                ## Do some handling or more feedback to user
    
    img_path = "docs/images/readme_spotstatemachine2.png"
    spot._graph().write_png(img_path)

