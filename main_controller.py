""" ARAP Webots main file """
import robot

            
def main():
    range = 0.0
    robot1 = robot.ARAP()
    robot1.init_devices()
    have_seen_blue = False
    have_seen_red = False
    have_seen_green = False    
    block_count = []

    while True:
        robot1.reset_actuator_values()
        range = robot1.get_sensor_input()
        robot1.blink_leds()
        
        def msg():
            nonlocal have_seen_blue
            nonlocal have_seen_red 
            nonlocal have_seen_green 
            nonlocal block_count
            red, green, blue = robot1.get_camera_image(5)
            
            if not have_seen_red :
                if 129 < red < 255 and 30 < green < 118 and 32 < blue < 118:
                    print("I see red block")
                    block_count.append("Red Block")        
                    print("Summary: I have seen ", block_count) 
                    have_seen_red = True
                
                    
            
            if not have_seen_blue:
                if 50 < red < 133 and 50 < green < 115 and 135 < blue < 255:
                
                    print("I see blue block")
                    block_count.append("Blue Block")
                    print("Summary: I have seen ", block_count)
                    have_seen_blue = True
            
                
            
            if not have_seen_green:
                if 50 < red < 116 and 129 < green < 255 and 50 < blue < 110:
                    print("I see green block")  
                    have_seen_green = True
                    block_count.append("Green Block")
                    print("Summary: I have seen ", block_count)  
                    have_seen_green = True
                    
            
                
            
    
        msg()
        
        if robot1.front_obstacles_detected():
            robot1.move_backward()
            robot1.turn_right()
            
        elif robot1.back_obstacles_detected():
            robot1.move_forward()
            
        elif robot1.left_obstacles_detected():
            robot1.turn_right()
            
        elif robot1.right_obstacles_detected():
            robot1.turn_left()
        
        else:
            robot1.run_braitenberg()
            #robot1.move_forward()
            
            
        robot1.set_actuators()
        robot1.step()
        


if __name__ == "__main__":
    main()
   
