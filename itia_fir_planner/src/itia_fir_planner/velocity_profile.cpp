#include <itia_fir_planner/velocity_profile.h>

namespace itia{
namespace helios{

VelocityProfile::VelocityProfile(const Eigen::VectorXd& qini, const unsigned int& np, const unsigned int& nfir, const double& st)
{
  m_t_last_change=0;
  m_t_2last_change=0;
  
  m_np = np;
  m_nfir = nfir;
  if (m_nfir<2)
  {
    ROS_WARN("FIR coeff should be greater or equal than 2, set equal to 2");
    m_nfir = 2;
  }
  
  m_st = st;
  m_nax = qini.rows();
  
  m_q_nf.resize(  m_np+m_nfir-1, m_nax);
  m_Dq_nf.resize( m_np+m_nfir-1, m_nax);
  m_q.resize(  m_np, m_nax);
  m_Dq.resize( m_np, m_nax);
  m_DDq.resize(m_np, m_nax);
  
  for (unsigned int iax = 0;iax<m_nax;iax++)
  {
    m_q.col(iax)    = Eigen::VectorXd::Constant(m_np, 1, qini(iax));
    m_q_nf.col(iax) = Eigen::VectorXd::Constant(m_np+m_nfir-1, 1, qini(iax));
  }
  
  m_Dq_nf.setZero();
  m_Dq.setZero();
  m_DDq.setZero();
  
  m_waypoints.resize(m_nax, 1);
  m_waypoints = qini;

  m_waypoints_vel.resize(m_nax, 1);
  m_waypoints_vel.setConstant(1);
  
  m_waypoints_time.resize(1);
  m_waypoints_time(0) = 0;
  
  m_waypoints_accuracy.resize(1);
  m_waypoints_accuracy(0) = 0.1;
  
  m_upper_limit.resize(m_nax);
  m_lower_limit.resize(m_nax);
  
  m_upper_limit.setConstant( 1000);
  m_lower_limit.setConstant(-1000);
  
  
  m_time = 0;
  m_actual_wp_idx = 0;
  m_last_wp_idx = 0;
  
  m_soft_stopped = false;
  m_last_Dq = Eigen::VectorXd::Constant(m_nax, 1, 0.0);
  
  m_vel_scale=1;
}

void VelocityProfile::fillBuffer()
{
  unsigned int wp_idx = m_actual_wp_idx;
  for (unsigned int ip = 0; ip<m_np;ip++)
  {
    double remaining_time = std::max(m_waypoints_time(wp_idx) - m_time-m_st*ip, m_st);

    
    Eigen::VectorXd vel  = ( m_waypoints.col(wp_idx)  - m_q_nf.row(ip+m_nfir-2).transpose() ).array() / remaining_time;
    
    for (unsigned int iax = 0;iax<m_nax;iax++)
    {
      vel(iax) = std::min( std::max( vel(iax), m_Dq_nf(0, iax)-m_waypoints_vel(iax, wp_idx)),  m_Dq_nf(0, iax)+m_waypoints_vel(iax, wp_idx));
      vel(iax) = std::min( std::max( vel(iax), -m_waypoints_vel(iax, wp_idx)),  m_waypoints_vel(iax, wp_idx));
    }

    m_Dq_nf.row(ip+m_nfir-1) = vel.transpose();
    m_q_nf.row( ip+m_nfir-1) = m_q_nf.row(ip+m_nfir-2) + vel.transpose() * m_st;
    
    for (unsigned int iax = 0;iax<m_nax;iax++)
    {
      m_q(ip, iax)  = m_q_nf.block( ip, iax, m_nfir, 1).mean();
      m_Dq(ip, iax) = m_Dq_nf.block(ip, iax, m_nfir, 1).mean();
    }
    if (ip>0)
      m_DDq.row(ip) = (m_Dq.row(ip)-m_Dq.row(ip-1)).array()/m_st;
    else
      m_DDq.row(ip) = m_Dq.row(ip).array()/m_st;
    
    if ( ( m_q.row(ip).transpose() - m_waypoints.col(wp_idx) ).array().abs().maxCoeff() < m_waypoints_accuracy(wp_idx) )
      if ( wp_idx < ( m_waypoints_time.rows() -1 ) )
        wp_idx++;
  }
  m_last_wp_idx = wp_idx;
  
}

void VelocityProfile::addPoints(const Eigen::VectorXd& time, const Eigen::MatrixXd& waypoints, const Eigen::MatrixXd& waypoints_vel, const Eigen::VectorXd& accuracy)
{
  m_waypoints_time     = time.array()+m_time;
  m_waypoints          = waypoints;
  m_waypoints_accuracy = accuracy;
  m_waypoints_vel      = waypoints_vel;
  
  m_t_last_change=m_time-(2.0*((double)m_nfir)*m_st);
  m_t_2last_change=m_time-(2.0*((double)m_nfir)*m_st);
  
  
  
  m_actual_wp_idx = 0;
  m_last_wp_idx = 0;
//   fillBuffer();
}


int VelocityProfile::update( double& time, Eigen::VectorXd& q, Eigen::VectorXd& Dq, Eigen::VectorXd& DDq )
{
  
  int return_code=0;
  try 
  {
    
    q   = m_q.row(0).transpose();
    Dq  = m_Dq.row(0).transpose();
    DDq = (m_Dq.row(0)-m_last_Dq).transpose()/m_st;
    m_last_Dq = Dq;
    time = m_time;
    
//     while ( m_actual_wp_idx < (m_waypoints_time.rows() -1 ))
//     {
//       if ( ( q - m_waypoints.col(m_actual_wp_idx) ).array().abs().maxCoeff() < std::max(m_waypoints_accuracy(m_actual_wp_idx), 1e-4))
//       {
//         m_actual_wp_idx++;
//         ROS_INFO("reached waypoint #%d/%zu", m_actual_wp_idx, m_waypoints_time.rows());
//       }
//       else
//         break;
//     }
    
    m_q.block(  0, 0, m_np-1, m_nax) = m_q.block(  1, 0, m_np-1, m_nax);
    m_Dq.block( 0, 0, m_np-1, m_nax) = m_Dq.block( 1, 0, m_np-1, m_nax);
    m_DDq.block(0, 0, m_np-1, m_nax) = m_DDq.block(1, 0, m_np-1, m_nax);
    
    m_q_nf.block(0, 0, m_np+m_nfir-2, m_nax)  = m_q_nf.block( 1, 0, m_np+m_nfir-2, m_nax);
    m_Dq_nf.block(0, 0, m_np+m_nfir-2, m_nax) = m_Dq_nf.block(1, 0, m_np+m_nfir-2, m_nax);
    
    
    unsigned int ip =  m_np-1;
    double remaining_time = std::max(m_waypoints_time(m_last_wp_idx) - m_time-m_st*ip, m_st);
    //ROS_INFO("remaining_time=%f", remaining_time);
    Eigen::VectorXd vel  = ( m_waypoints.col(m_last_wp_idx)  - m_q_nf.row(ip+m_nfir-2).transpose() ).array() / remaining_time;
    Eigen::VectorXd vel_limited=vel;

    double scale=1.0;
    
    for (unsigned int iax = 0;iax<m_nax;iax++)
    {
      vel_limited(iax) = std::min( std::max( vel_limited(iax), m_Dq_nf(ip-1, iax)-m_waypoints_vel(iax, m_last_wp_idx)),  m_Dq_nf(ip-1, iax)+m_waypoints_vel(iax, m_last_wp_idx));
      vel_limited(iax) = std::min( std::max( vel_limited(iax), -m_waypoints_vel(iax, m_last_wp_idx)),  m_waypoints_vel(iax, m_last_wp_idx));
      if (   (vel(iax)!=0) )
        scale = std::min( scale, std::abs( vel_limited(iax)/vel(iax) ) );
    }
    vel*=scale*m_vel_scale;
    
    bool is_on_limit=false;;
    Eigen::VectorXd next_pose = m_q_nf.row(ip+m_nfir-2) + vel.transpose() * m_st;
    for (unsigned int iax = 0;iax<m_nax;iax++)
    {
      
      bool upper_bound = (next_pose(iax)>=m_upper_limit(iax)) && (vel(iax)>0) ;
      bool lower_bound = (next_pose(iax)<=m_lower_limit(iax)) && (vel(iax)<0) ;
      is_on_limit = (is_on_limit) || ( upper_bound ) || ( lower_bound ) ;
    }
    if (is_on_limit)
    {
      return_code=-1;
      vel.setZero();
    }
    
    m_Dq_nf.row(ip+m_nfir-1) = vel.transpose();
    m_q_nf.row( ip+m_nfir-1) = m_q_nf.row(ip+m_nfir-2) + vel.transpose() * m_st;
    m_time += m_st;
    
    for (unsigned int iax = 0;iax<m_nax;iax++)
    {
      m_q(ip, iax)  = m_q_nf.block( ip, iax, m_nfir, 1).mean();
      m_Dq(ip, iax) = m_Dq_nf.block(ip, iax, m_nfir, 1).mean();
    }
    
    
    if (ip>0)
      m_DDq.row(ip) = (m_Dq.row(ip)-m_Dq.row(ip-1)).array()/m_st;
    else
      m_DDq.row(ip) = m_Dq.row(ip).array()/m_st;
    
    m_soft_stopped =false;
    if ( ( (m_time-m_t_2last_change)> (0.25*((double)m_nfir)*m_st) ) && ( m_last_wp_idx < ( m_waypoints_time.rows() -1 ) ) )
    {
      if ( ( m_q_nf.row(ip+m_nfir-1).transpose() - m_waypoints.col(m_last_wp_idx) ).array().abs().maxCoeff() <= std::max(m_waypoints_accuracy(m_last_wp_idx),1e-4) )
      {
        m_t_2last_change=m_t_last_change;
        m_t_last_change=m_time;
        m_last_wp_idx++;
        ROS_DEBUG("reached waypoint #%d/%zu", m_last_wp_idx, m_waypoints_time.rows());
      }
    }
    else
    {
      if ( ( m_q.row(ip).transpose() - m_waypoints.col(m_last_wp_idx) ).array().abs().maxCoeff() <= std::max(m_waypoints_accuracy(m_last_wp_idx),1e-4) )
      {
        if ( m_last_wp_idx < ( m_waypoints_time.rows() -1 ) )
        {
          m_last_wp_idx++;
          m_t_2last_change=m_t_last_change;
          m_t_last_change=m_time;
          ROS_DEBUG("reached waypoint #%d/%zu", m_last_wp_idx, m_waypoints_time.rows());
        }
        else
          return 1;
      }
    }
  }
  catch (std::exception& e)
  {
    ROS_ERROR("VelocityProfile Update error: %s",e.what());
    q   = m_q.row(0).transpose();
    Dq.setZero();
    DDq.setZero();
    return -2;
  }
  catch (...)
  {
    ROS_ERROR("VelocityProfile Update unchatched error");
    q   = m_q.row(0).transpose();
    Dq.setZero();
    DDq.setZero();
    return -2;
  }
  
  return return_code;
}


void VelocityProfile::updateVel ( const Eigen::VectorXd& input_vel, double& time, Eigen::VectorXd& q, Eigen::VectorXd& Dq, Eigen::VectorXd& DDq )
{
  try
  {
    q   = m_q.row(0).transpose();
    Dq  = m_Dq.row(0).transpose();
    DDq = (m_Dq.row(0)-m_last_Dq).transpose()/m_st;
    m_last_Dq = Dq;
    time = m_time;
    
    m_q.block(  0, 0, m_np-1, m_nax) = m_q.block(  1, 0, m_np-1, m_nax);
    m_Dq.block( 0, 0, m_np-1, m_nax) = m_Dq.block( 1, 0, m_np-1, m_nax);
    m_DDq.block(0, 0, m_np-1, m_nax) = m_DDq.block(1, 0, m_np-1, m_nax);
    
    m_q_nf.block(0, 0, m_np+m_nfir-2, m_nax)  = m_q_nf.block( 1, 0, m_np+m_nfir-2, m_nax);
    m_Dq_nf.block(0, 0, m_np+m_nfir-2, m_nax) = m_Dq_nf.block(1, 0, m_np+m_nfir-2, m_nax);
    
    unsigned int ip =  m_np-1;
    Eigen::VectorXd vel  = input_vel;
    Eigen::VectorXd vel_limited=vel;
    
    double scale=1.0;
    
    
    for (unsigned int iax = 0;iax<m_nax;iax++)
    {
      vel_limited(iax) = std::min( std::max( vel_limited(iax), m_Dq_nf(ip-1, iax)-m_waypoints_vel(iax, m_last_wp_idx)),  m_Dq_nf(ip-1, iax)+m_waypoints_vel(iax, m_last_wp_idx));
      vel_limited(iax) = std::min( std::max( vel_limited(iax), -m_waypoints_vel(iax, m_last_wp_idx)),  m_waypoints_vel(iax, m_last_wp_idx));
      if (   (vel(iax)!=0) )
        scale = std::min( scale, std::abs( vel_limited(iax)/vel(iax) ) );
    }
    vel*=scale*m_vel_scale;
    
    m_Dq_nf.row(ip+m_nfir-1) = vel.transpose();
    m_q_nf.row( ip+m_nfir-1) = m_q_nf.row(ip+m_nfir-2) + vel.transpose() * m_st;
    m_time += m_st;
    
    for (unsigned int iax = 0;iax<m_nax;iax++)
    {
      m_q(ip, iax)  = m_q_nf.block( ip, iax, m_nfir, 1).mean();
      m_Dq(ip, iax) = m_Dq_nf.block(ip, iax, m_nfir, 1).mean();
    }
    
    
    if (ip>0)
      m_DDq.row(ip) = (m_Dq.row(ip)-m_Dq.row(ip-1)).array()/m_st;
    else
      m_DDq.row(ip) = m_Dq.row(ip).array()/m_st;
    
    m_soft_stopped =false;
  }
  catch (std::exception& e)
  {
    ROS_ERROR("VelocityProfile Update error: %s",e.what());
    q   = m_q.row(0).transpose();
    Dq.setZero();
    DDq.setZero();
  }
  catch (...)
  {
    ROS_ERROR("VelocityProfile Update unchatched error");
    q   = m_q.row(0).transpose();
    Dq.setZero();
    DDq.setZero();
  }
  
  return;
  
}

void VelocityProfile::softStop(double& time, Eigen::VectorXd& q, Eigen::VectorXd& Dq, Eigen::VectorXd& DDq)
{
  for (unsigned int ip =  0;ip<m_np;ip++)
  for (unsigned int iax = 0;iax<m_nax;iax++)
  {
    m_Dq(ip, iax) = m_Dq_nf.block(ip, iax, m_nfir, 1).mean();
    m_q(ip, iax)  = m_q_nf.block( ip, iax, m_nfir, 1).mean();
  }
  q   = m_q.row(0).transpose();
//   q.setZero();
  Dq  = m_Dq.row(0).transpose();
  DDq = (m_Dq.row(0)-m_last_Dq).transpose()/m_st;
  m_last_Dq = Dq;
  time = m_time;
  
  for (unsigned int iax = 0;iax<m_nax;iax++)
  {
  //  m_q_nf.col(iax) = m_q_nf.col(iax).array()+Dq(iax) *m_st;
    Eigen::VectorXd Dq_nf(m_nax);
    Dq_nf.setZero();
    for (unsigned int ifir = 0;ifir<(m_np+m_nfir-1);ifir++)
    {
      if (m_Dq_nf(ifir, iax) != 0)
      {
        if ( (m_Dq_nf(ifir, iax)>0) )
        {
          Dq_nf(iax)=m_Dq_nf(ifir, iax);
          m_Dq_nf(ifir, iax) = 0;
          
          if ( m_Dq(0, iax) >= 0 )
            break;
        }
        else if ( (m_Dq_nf(ifir, iax) < 0))
        {
          Dq_nf(iax)=m_Dq_nf(ifir, iax);
          m_Dq_nf(ifir, iax) = 0;
          if ( m_Dq(0, iax) <= 0 )
            break;
        }
      }
    }
    ROS_INFO_STREAM("Dq_nf="<<Dq_nf.transpose());
    m_q_nf.col(iax) = m_q_nf.col(iax).array()+Dq_nf(iax) *m_st;
    
    
  }

}

void VelocityProfile::changeVelocityScale(const double& scale)
{
  if (scale>100)
    m_vel_scale=1;
  else if ( scale<0 )
    m_vel_scale=0;
  else
    m_vel_scale=scale/100.0;
  
}


}
}