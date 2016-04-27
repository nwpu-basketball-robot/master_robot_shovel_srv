/*
*Team Unware Basketball Robot NWPU
*
*用于接收其他节点的铲子控制请求，并向下位机发送控制指令
*
*Author = liao-zhihan
*
*first_debug_date:2015-07
*测试通过:date 2015-07
*
*second_debug_date:2016-01-20
*改善了代码规范
*测试通过:date 2016-04
*/

#include <ros/ros.h>
#include <basketball_msgs/basketball_shovel_srv.h>
#include <basketball_msgs/robot_message.h>

class RobotShovelSrv
{
public:
    RobotShovelSrv(ros::NodeHandle &node) ;
    ~RobotShovelSrv() ;
protected:
private:
    //向下位机发送铲子控制请求
    bool pubShovelCmd(const uint8_t func , const uint8_t control_id) ;
    //回调函数
    bool shovelSrvCallBack(basketball_msgs::basketball_shovel_srv::Request &req ,       basketball_msgs::basketball_shovel_srv::Response &rep) ;

    //todo:
    //增加铲子下位机的监视程序
    void changeCurrentShovelControlType(int control_type) ;

private:
    enum ShovelControlType{SHOVEL_ON,SHOVEL_DOWN,SHOVEL_CATCHBALL,SHOVEL_AFTER_CATCH_DOWN} current_control_type_ ;//控制方法
    ros::ServiceServer robot_shovel_srv_ ;
    ros::NodeHandle nh_ ;
    ros::Publisher robot_shovel_pub_ ;

    int shovel_cmd_id_ ;
};

RobotShovelSrv::RobotShovelSrv(ros::NodeHandle &node)
    :nh_(node),
     shovel_cmd_id_(0x02)
{
    robot_shovel_pub_ = nh_.advertise<basketball_msgs::robot_message>("robot_cmd",1000) ;
    robot_shovel_srv_ = nh_.advertiseService("cmd_shovel",&RobotShovelSrv::shovelSrvCallBack,this) ;
}

RobotShovelSrv::~RobotShovelSrv()
{
    nh_.shutdown() ;
}

bool RobotShovelSrv::pubShovelCmd(const uint8_t func , const uint8_t control_id)
{
    //协议转换
    basketball_msgs::robot_message robot_cmd_msg ;
    robot_cmd_msg.data.resize(7 , 0) ;
    uint8_t *data_ptr = robot_cmd_msg.data.data() ;
    int data_len = 2 ;
    data_ptr[0] = data_ptr[1] = 0xff ;
    data_ptr[2] = shovel_cmd_id_ ;
    data_ptr[3] = (u_int8_t)(data_len>>8) ;
    data_ptr[4] = (u_int8_t)(data_len & 0xff) ;
    data_ptr[5] = func ;
    data_ptr[6] = control_id ;
    //协议转换完成
    robot_shovel_pub_.publish(robot_cmd_msg) ;
    return true ;
}

void RobotShovelSrv::changeCurrentShovelControlType(int control_type)
{
    switch (control_type) {
    case 0:
        current_control_type_ = SHOVEL_ON ; //铲子从最低处到达最高点
        break ;
    case 1:
        current_control_type_ = SHOVEL_DOWN ; //铲子从最高点到最低点
        break ;
    case 2:
        current_control_type_ = SHOVEL_CATCHBALL ; //铲子把球放入弹射装置，并且放下
        break ;
    case 3:
        current_control_type_ = SHOVEL_AFTER_CATCH_DOWN ;//铲子将球抬在空中（按比赛要求，已经取消该控制方式）
        break ;
    }
}

bool RobotShovelSrv::shovelSrvCallBack(basketball_msgs::basketball_shovel_srv::Request &req,
                                  basketball_msgs::basketball_shovel_srv::Response &rep)
{
    changeCurrentShovelControlType(req.order_type) ;
    rep.is_successed = pubShovelCmd(0x01,uint8_t(current_control_type_+1)) ;
    return true ;
}

int main(int argc , char **argv)
{
    ros::init(argc , argv , "shovel_service")  ;
    ros::NodeHandle node ;
    RobotShovelSrv shovel_srv(node) ;
    ros::spin() ;
}
