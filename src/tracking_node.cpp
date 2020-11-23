#include "../include/tracking.h"

using namespace std;

void trackingCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
  int i = 0, j=0;
	//int pkg_0[240];

	memset(tracking_data,0,sizeof(tracking_data));
	//Float to ınteger Conversion
  for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{		
		

			tracking_data[i] = static_cast<int>((*it * 1000) +std::pow(2,16));


		    
 	 		printf("%2.3f ",*it);

		i++;
	}


 	 printf("\n");

	sendto(senderSocket, tracking_data, sizeof(tracking_data), 0, (struct sockaddr *)&receiver, sizeof(receiver));

}

int main(int argc, char **argv)
{


  for(int i=0; i<320; i++)
		{
			temp[i] =0;
		}
  //Create UDP socket
  senderSocket = socket(AF_INET, SOCK_DGRAM, 0);

  //Configure settings in address struct
  receiver.sin_family = AF_INET;
  receiver.sin_port = htons(54204);  //receiver port no
  receiver.sin_addr.s_addr = inet_addr("192.168.140.99"); //receiver ip addr
  memset(receiver.sin_zero, '\0', sizeof(receiver.sin_zero));  
  memset(tracking_data, 0, sizeof(tracking_data));
 
  //Configure settings in address struct
  sender.sin_family = AF_INET;
  sender.sin_port = htons(54203);  //sender fixed port no
  sender.sin_addr.s_addr = inet_addr("192.168.140.95"); //sender ip addr
  memset(sender.sin_zero, '\0', sizeof(sender.sin_zero));  

  bind(senderSocket,(struct sockaddr *)&sender,sizeof(sender));

  ros::init(argc, argv, "tracking_node");
  ros::NodeHandle n;
  ros::Subscriber radar_sub = n.subscribe("/Objects_merge", 100, trackingCallback);
  sendto(senderSocket, temp, sizeof(temp), 0, (struct sockaddr *)&receiver, sizeof(receiver));
  ros::spin();
  return 0;
}