  if self.distances['front']['static'] < 1.0:
                if self.distances['left']['static'] < 1.0:
                    while(self.obstacle=='YES'):

                        self.get_logger().info("Obstacle detected in left!")
                        linear_x = 0.0
                        linear_y = -0.1
                        angular_z = 0.0
                        self.send_command(linear_x, linear_y, angular_z)  # Send stop command
                        if self.distances['front']['static'] > 1.0 and self.distances['Northeast']['static'] > 1.0:
                            self.get_logger().info("NO Obstacle F and NE detected !")
                            self.obstacle='NO'
                  
                if self.distances['right']['static'] < 1.5:
                    while(self.obstacle=='Yes'):

                        self.get_logger().info("Obstacle detected in font and right!")
                        linear_x = 0.0
                        linear_y = 0.1
                        angular_z = 0.0
                        self.send_command(linear_x, linear_y, angular_z)  # Send stop command
                        if self.distances['front']['static'] > 1.0 and self.distances['Northwest']['static'] > 1.0:
                            self.get_logger().info("NO Obstacle F and NW detected !")
                            self.obstacle='NO'

                    
                    
                if self.distances['left']['static'] < 1.0 and self.distances['right']['static'] < 1.0:
                    while(self.obstacle=='Yes'):
                        self.get_logger().info("Obstacle detected in robot! Stopping the robot.")
                        linear_x = 0.0
                        linear_y = 0.0
                        angular_z = 0.0
                        self.send_command(linear_x, linear_y, angular_z)  # Send stop command
                        if self.distances['left']['static'] > 1.0 or self.distances['right']['static'] > 1.0:
                            self.get_logger().info("NO Obstacle L and R detected !")
                            self.obstacle='NO'
