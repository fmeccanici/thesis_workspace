#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <slave_control/parametersConfig.h>

double Kpm_x, Kpm_y, Kpm_z, Kdm_x, Kdm_y, Kdm_z, Kfm_x, Kfm_y ,Kfm_z;
double Kps_x, Kps_y, Kps_z, Kds_x, Kds_y, Kds_z, Kfs_x, Kfs_y ,Kfs_z;
double b_x, b_y, b_z, CSel, WVfilt_x, WVfilt_y, WVfilt_z, PClim_x, PClim_y, PClim_z;
double velfilt_m_x, velfilt_m_y, velfilt_m_z, velfilt_s_x, velfilt_s_y, velfilt_s_z;
double forcefilt_e_x, forcefilt_e_y, forcefilt_e_z, forcefilt_h_x, forcefilt_h_y, forcefilt_h_z;
double forcefilt_m_x, forcefilt_m_y, forcefilt_m_z, forcefilt_s_x, forcefilt_s_y, forcefilt_s_z;
double posloopfilt_s_x, posloopfilt_s_y, posloopfilt_s_z;

std::vector<double> c_param(49);


void callback(slave_control::parametersConfig &config, uint32_t level, ros::NodeHandle n) 
{
  
    Kpm_x = config.Kpm_x; Kpm_y = config.Kpm_y; Kpm_z = config.Kpm_z;
    Kdm_x = config.Kdm_x; Kdm_y = config.Kdm_y; Kdm_z = config.Kdm_z;
    Kfm_x = config.Kfm_x; Kfm_y = config.Kfm_y; Kfm_z = config.Kfm_z;
    Kps_x = config.Kps_x; Kps_y = config.Kps_y; Kps_z = config.Kps_z;
    Kds_x = config.Kds_x; Kds_y = config.Kds_y; Kds_z = config.Kds_z;  
    Kfs_x = config.Kfs_x; Kfs_y = config.Kfs_y; Kfs_z = config.Kfs_z;
    
    b_x = config.b_x; b_y = config.b_y; b_z = config.b_z;

    CSel = config.CSel;
    
    WVfilt_x = config.WVfilt_x; WVfilt_y = config.WVfilt_y; WVfilt_z = config.WVfilt_z;
    PClim_x = config.PClim_x; PClim_y = config.PClim_y; PClim_z = config.PClim_z;
    
    velfilt_m_x = config.velfilt_m_x; velfilt_m_y = config.velfilt_m_y; velfilt_m_z = config.velfilt_m_z;
    velfilt_s_x = config.velfilt_s_x; velfilt_s_y = config.velfilt_s_y; velfilt_s_z = config.velfilt_s_z;

    forcefilt_e_x = config.forcefilt_e_x; forcefilt_e_y = config.forcefilt_e_y; forcefilt_e_z = config.forcefilt_e_z;
    forcefilt_h_x = config.forcefilt_h_x; forcefilt_h_y = config.forcefilt_h_y; forcefilt_h_z = config.forcefilt_h_z;
    forcefilt_m_x = config.forcefilt_m_x; forcefilt_m_y = config.forcefilt_m_y; forcefilt_m_z = config.forcefilt_m_z;
    forcefilt_s_x = config.forcefilt_s_x; forcefilt_s_y = config.forcefilt_s_y; forcefilt_s_z = config.forcefilt_s_z;

    posloopfilt_s_x = config.posloopfilt_s_x; posloopfilt_s_y = config.posloopfilt_s_y; posloopfilt_s_z = config.posloopfilt_s_z;
  
    c_param[0] = Kpm_x; c_param[1] = Kpm_y; c_param[2] = Kpm_z;
    c_param[3] = Kdm_x; c_param[4] = Kdm_y; c_param[5] = Kdm_z;
    c_param[6] = Kfm_x; c_param[7] = Kfm_y; c_param[8] = Kfm_z;

    c_param[9] = Kps_x; c_param[10] = Kps_y; c_param[11] = Kps_z;
    c_param[12] = Kds_x; c_param[13] = Kds_y; c_param[14] = Kds_z;
    c_param[15] = Kfs_x; c_param[16] = Kfs_y; c_param[17] = Kfs_z;
    
    c_param[18] = b_x; c_param[19] = b_y; c_param[20] = b_z; 

    c_param[21] = CSel;
    
    c_param[22] = WVfilt_x; c_param[23] = WVfilt_y; c_param[24] = WVfilt_z;
    c_param[25] = PClim_x; c_param[26] = PClim_y; c_param[27] = PClim_z;

    c_param[28] = velfilt_m_x; c_param[29] = velfilt_m_y; c_param[30] = velfilt_m_z;
    c_param[31] = velfilt_s_x; c_param[32] = velfilt_s_y; c_param[33] = velfilt_s_z;

    c_param[34] = forcefilt_e_x; c_param[35] = forcefilt_e_y; c_param[36] = forcefilt_e_z;
    c_param[37] = forcefilt_h_x; c_param[38] = forcefilt_h_y; c_param[39] = forcefilt_h_z;
    c_param[40] = forcefilt_m_x; c_param[41] = forcefilt_m_y; c_param[42] = forcefilt_m_z;
    c_param[43] = forcefilt_s_x; c_param[44] = forcefilt_s_y; c_param[45] = forcefilt_s_z;

    c_param[46] = posloopfilt_s_x; c_param[47] = posloopfilt_s_y; c_param[48] = posloopfilt_s_z;

   n.setParam("c_param", c_param);
  
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "haptic_control_parameter_server_m");
    ros::NodeHandle n;
    dynamic_reconfigure::Server<slave_control::parametersConfig> server;
    dynamic_reconfigure::Server<slave_control::parametersConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2, n);
    server.setCallback(f);

    ROS_INFO("Starting dynamic parameter server node");
    ros::spin();
    return 0;
}