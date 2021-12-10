// Run with: $ roslaunch second_assignment second_assignment.launch

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/Service.h"
#include "std_srvs/Empty.h"
#include "stdio.h"

// Initialize a raw string literal to print the menu
const char * menu = R"(
   MENU
   --------------------------------------------
   + : increase speed
   - : decrease speed
   r : reset robot position
   q : quit
)";


int main ( int argc, char ** argv ) {

   float velocity = 0.0;
   float tmp;
   
   // Initialize ros node
   ros::init(argc, argv, "command_interface");
   ros::NodeHandle nh;

   // Create a client for my service
   ros::ServiceClient client_srv = nh.serviceClient<second_assignment::Service>("/service");

   // Creates a client to reset the position of the robot
   ros::ServiceClient client_res = nh.serviceClient<std_srvs::Empty>("/reset_positions");

   // Initialize servers for necessary services
   second_assignment::Service server_srv;
   std_srvs::Empty server_res;

   // Print menu and actual velocity on the screen
   printf( "%s", menu );
   printf( "Speed: %f\n", velocity);

   char key = 'r'; // Initialize the variable key

   //While the roscore is running, execute the loop
   while (ros::ok()) {

      scanf(" %c", &key); // Read command
      
      if(key == 'q')
      {
      	ros::shutdown();
      }
      else if ( key == '+' || key == '-' || key == 'r') 
      {
         // Valid command received
         // Send a request to the service
         server_srv.request.command = key;
	 client_srv.waitForExistence();

         if ( client_srv.call(server_srv) ) 
         {

            if ( key == 'r' ) 
            {
            	// Reset invoked
               client_res.waitForExistence();
               client_res.call(server_res);
               printf("RESET\n");
            }
            
            // Receive the new velociity from the server
	    tmp = server_srv.response.new_velocity;
            // Print if velocity doesn't change
            if(tmp == 0 && velocity == 0 && key != 'r')
            	printf("The velocity can't be decreased!\n");
            else if(tmp == velocity && tmp == 10)
            	printf("The velocity can't be encreased!\n");
            else
            	velocity = tmp;
            // Print new velocity
            printf( "Speed: %f\n", velocity);
         }
      } else {
         // Print if the command is not valid
         printf( "Invalid command: %c\n", key );
      }
   }

   return 0;
}
