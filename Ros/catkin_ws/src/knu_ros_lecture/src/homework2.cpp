// 최연제 2014105092 5월 29 일 

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <stdio.h>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))
#define PI 3.14159265358979323846



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 전역변수 
boost::mutex mutex;
nav_msgs::Odometry g_odom;
float pre_dAngleTurned;


double dRotation ;
float _dRatation ;
double dTranslation ;
double x, y;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 콜백함수 
void
odomMsgCallback(const nav_msgs::Odometry &msg)
{
    // 뮤텍스 락을 걸어서 /odom 토픽의 메세지를 받습니다.
    mutex.lock(); {
        g_odom = msg;
    } mutex.unlock();
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
tf::Transform
getCurrentTransformation(void)
{
    // transformation 버퍼 
    tf::Transform transformation;

    // odom 버퍼 
    nav_msgs::Odometry odom;

    // copy a global '/odom' message with the mutex
    // 락을 건후 /odom 의 토픽 메세지를 전역변수로 선언 
    mutex.lock(); {
        odom = g_odom;
    } mutex.unlock();

    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;
    // 위치를 저장 
    transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

    // 회전각 저장 
    transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

    // 반환 
    return transformation;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
tf::Transform
getInitialTransformation(void)
{
    //transformation 버퍼 
    tf::Transform transformation;

    // 루프 회전 주기를 정함 
    ros::Rate loopRate(1000.0);

    while(ros::ok()) {
        // 콜백함수 호출 
        ros::spinOnce();

        // get current transformationreturn;
	// 현재의 transformation 을 반환 
        transformation = getCurrentTransformation();
 
        // x,y,z 가 0이 다 아니면 즉 움직이고 있다면 루프 종료 후 현재 위치 반환 
        if(transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) {
            break;
        } else {
            loopRate.sleep();
        }
    }

    // 반환 
    return transformation;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 회전 
bool
doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
    // rad/s 로 rotationSpeed 를 바꿉니다. 
    geometry_msgs::Twist baseCmd;
    baseCmd.linear.x = 0.0;
    baseCmd.linear.y = 0.0;

    if(dRotation < 0.) {
        baseCmd.angular.z = -dRotationSpeed;
    } else {
        baseCmd.angular.z = dRotationSpeed;
    }

    bool bDone = false;
    ros::Rate loopRate(1000.0);



    while(ros::ok() && !bDone) {
        // ÀÏŽÜ callback žÞœÃÁöžŠ ¹Þ°í!

        ros::spinOnce();

        // 현재 tf 를 받음 
        tf::Transform currentTransformation = getCurrentTransformation();

        //얼마나 이동했는지 확인 
        tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
        tf::Quaternion rotationQuat = relativeTransformation.getRotation();


        
         double dAngleTurned = atan2((2 * rotationQuat[2] * rotationQuat[3]) , (1-(2 * (rotationQuat[2] * rotationQuat[2]) ) ));

  // 회전 얼마나 했는지 확인 
    if( fabs(dAngleTurned) > fabs(dRotation) || (abs(pre_dAngleTurned - dRotation) <  abs(dAngleTurned - dRotation)) || (dRotation == 0)) 
	{
            bDone = true;
            break;
        } else {
	    pre_dAngleTurned = dAngleTurned;
            // 이동해라고 메세지 보냄 
            pubTeleop.publish(baseCmd);

            // sleep!
            loopRate.sleep();
        }
    }

    // baseCmd 를 초기화 한후 publish 시
    baseCmd.linear.x = 0.0;
    baseCmd.angular.z = 0.0;
    pubTeleop.publish(baseCmd);

    return bDone;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
bool
doTranslation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dTranslation, double dTranslationSpeed)
{
    //the command will be to go forward at 'translationSpeed' m/s
    // 속도의 단위를 m/s 로 바꿔줍니다.
    geometry_msgs::Twist baseCmd;

    if(dTranslation < 0) {
        baseCmd.linear.x = -dTranslationSpeed;
    } else {
        baseCmd.linear.x = dTranslationSpeed;
    }

    baseCmd.linear.y = 0;
    baseCmd.angular.z = 0;

    bool bDone = false;
    ros::Rate loopRate(1000.0);

    while(ros::ok() && !bDone) {
        // 콜백함수 요청 
        ros::spinOnce();

        // 현재 tf 를 받음 
        tf::Transform currentTransformation = getCurrentTransformation();

        //얼마나 이동했는지 확인 
        tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
        double dDistMoved = relativeTransformation.getOrigin().length();

        // 얼마나 앞으로 갔는지 확인 (속도조절 )

        if(fabs(dDistMoved) >= fabs(dTranslation)) {
            bDone = true;
            break;
        } else {
            //send the drive command
            pubTeleop.publish(baseCmd);

            // sleep!
            loopRate.sleep();
        }
    }

    //  초기화후 publish

    baseCmd.linear.x = 0.0;
    baseCmd.angular.z = 0.0;
    pubTeleop.publish(baseCmd);

    return bDone;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int main(int argc, char **argv)
{
    // turtle_position_move 라는 노드 생성 
    ros::init(argc, argv, "turtle_position_move");

    // Ros initialization
    ros::NodeHandle nhp, nhs;

    //  subscriber 선언 
    ros::Subscriber sub = nhs.subscribe("/odom", 100, &odomMsgCallback);

    // publisher 객체 생성 
    ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	FILE *pfile = NULL;

	pfile = fopen("hw2.txt","r");
	if(pfile == NULL){		
		return 0;
	}
	
	
    
    int n,i;
    double x1, y1;
    double beforevalue , temp ;
    // 가는 각도 - 원래각도

    fscanf(pfile,"%d",&n);
    printf("%d\n",n);
    
    for(i = 0; i < n ; i++){

    fscanf(pfile,"%lf%lf",&x1,&y1);
    

    // parameter 로 회전각과 translation 을 받음 ;
    // 처음 위치 정보를 얻음 
    tf::Transform initialTransformation = getInitialTransformation();
    
    printf("\n%d 번쨰 \nx = %lf, y = %lf , x1 = %lf , y1 = %lf\n",i,x,y,x1,y1);

    printf("y1 - y = %lf , x1 - x = %lf , divide = %lf\n", y1-y , x1-x,(y1-y)/(x1-x) );

    //angle = atan(y/x) * 180/PI ;

    dRotation = atan((y1-y)/(x1-x)) * 180/PI ;
    //beforevalue = dRotation ;
    if(i == 0){
	beforevalue = dRotation;
    }
    else{
	// 1. 회전해야 되는 각 , 2. 다음을 위한 이전각 저장 , 3. 회전해야되는 
	// 2번 = beforevalue 대입
	temp = dRotation ;
	printf("이동전  각도 : %lf\n", beforevalue);
	dRotation = dRotation - beforevalue ;
	beforevalue = temp ;
	// 1. 회전해야 되는 각 = dRotation - angle
    }
    

    printf("atan value = %lf\n",dRotation);
    

    _dRatation = (float)((int) dRotation % 360);
    dTranslation = sqrt( pow((x1-x),2) + pow(y1-y,2) ) ;	
    
    

    if(abs(_dRatation) > 180){
          if(dRotation > 0) dRotation = -(360-_dRatation);
          else dRotation = (360+_dRatation); }
    else
         dRotation = _dRatation;

    printf("길이 : %lf , 각도 : %lf \n",dTranslation,dRotation);

    // turtlebot 을 회전을 시킴 

    doRotation(pub, initialTransformation, toRadian(dRotation), 0.75);

    // 전진을 시킴 (속도 조절 )

    doTranslation(pub, initialTransformation, dTranslation, 0.25);

    sleep(1);

    }

    fclose(pfile);

    return 0;
}
