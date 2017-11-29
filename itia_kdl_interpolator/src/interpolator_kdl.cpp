#include <itia_kdl_interpolator/interpolator_kdl.h>

unibs::deburring::InterpolatorKDL::InterpolatorKDL(ros::NodeHandle* ptr_nh): m_loop_rate( 1000 ), m_full_trj()
{  
	m_ptr_nh=ptr_nh;
	
	// Inizializzazione 
  bool found = true;
	std::vector<double> tmp;
	if(m_ptr_nh->getParam("/parameter_set/bframe",tmp))
		m_bframe = itia::gutils::stdVectorToArray(tmp);
	else
  {
		m_bframe.setZero();
    found=false;
  }
	
	if(m_ptr_nh->getParam("/parameter_set/uframe",tmp))
		m_uframe = itia::gutils::stdVectorToArray(tmp);
	else
  {
		m_uframe << 1.0, 0.0, 0.7, 0.0, 0.0, 0.0;
    found=false;
  }
	
	if(m_ptr_nh->getParam("/parameter_set/tframe",tmp))
		m_tframe = itia::gutils::stdVectorToArray(tmp);
	else
  {
		m_tframe << 0.130, 0.0, 0.075, PI, (120.0*PI/180.0), PI;
    found=false;
  }
	
	if(m_ptr_nh->getParam("/parameter_set/endstroke_neg",tmp))
		m_endstroke_neg = itia::gutils::stdVectorToArray(tmp);
	else
  {
		m_endstroke_neg << -PI, (-PI/3), (-170.0*PI/180.0), (-2700.0*PI/180.0), (-120.0*PI/180.0), (-2700.0*PI/180.0);
    found=false;
  }
	
	if(m_ptr_nh->getParam("/parameter_set/endstroke_pos",tmp))
		m_endstroke_pos = itia::gutils::stdVectorToArray(tmp);
	else
  {
		m_endstroke_pos << PI, (155.0*PI/180.0), (110.0*PI/180.0), (2700.0*PI/180.0), (120.0*PI/180.0), (2700.0*PI/180.0);
    found=false;
  }
	
	if(m_ptr_nh->getParam("/parameter_set/q_avg",tmp))
		m_q_avg = itia::gutils::stdVectorToArray(tmp);
	else 
  {
		m_q_avg = (m_endstroke_pos+m_endstroke_neg)/2;
		m_q_avg(4) = -PI/6;
		m_q_avg(2) = -PI/2;
    found=false;
	}
	if(!found)
    printf(" [ %s%s:%d%s ]\t %sMissing references definitions in the %s'/parameter_set'%s namespace. Set to default values.%s \n", 
           YELLOW, __FUNCFILE__, __LINE__, RESET,YELLOW,BLUE,YELLOW,RESET);
	
	std::string urdf_pkg_name, urdf_file_name;
	if(!m_ptr_nh->getParam("/parameter_set/urdf_pkg_name",urdf_pkg_name))
  {
		printf(" [ %s%s:%d%s ]\t %sImpossible to find the name of the package containing the URDF file in the parameter server!%s \n", 
           RED, __FUNCFILE__, __LINE__, RESET, RED, RESET);
		return;
	}
	if(!m_ptr_nh->getParam("/parameter_set/urdf_file_name",urdf_file_name))
  {
    printf(" [ %s%s:%d%s ]\t %sImpossible to find the name of the URDF file in the parameter server!%s \n", 
           RED, __FUNCFILE__, __LINE__, RESET, RED, RESET);
    return;
  }
  std::string urdf_path = ros::package::getPath(urdf_pkg_name);
  urdf_path = urdf_path + "/urdf/" + urdf_file_name;
	try
	{
		KDL::Tree my_tree;
		if (!kdl_parser::treeFromFile(urdf_path, my_tree))
    {
			ROS_ERROR("Failed to construct kdl tree");
			return;
		}
		my_tree.getChain("base_link","Flange",m_urdf_chain);
	} catch (std::exception e)
  {	
		std::cout << "Initialization failure: " << e.what() << std::endl;  
		throw;            
	}
	//std::cout << "Num giunti: " << m_urdf_chain.getNrOfJoints() << "; Num segmenti: " << m_urdf_chain.getNrOfSegments() << "\n";
	
	//aggiungo segmento fittizio per portarmi sul toolframe
	Eigen::Matrix4d matrix_frame;
	matrix_frame = itia::gutils::ArrayToMatrix(m_tframe);
	KDL::Frame flange_to_tool_kdl;
	flange_to_tool_kdl = EigenKDL::frame(matrix_frame);
	std::string name = "Tool_frame";
	KDL::Joint j("None");
	KDL::Segment tool(name,j,flange_to_tool_kdl);
	m_urdf_chain.addSegment(tool);
	//std::cout << "Num giunti: " << m_urdf_chain.getNrOfJoints() << "; Num segmenti: " << m_urdf_chain.getNrOfSegments() << "\n";
	
	// acquisisco posizione corrente del robot
	ros::Subscriber currentPos_subscriber = m_ptr_nh->subscribe("/act_robot_cart_pos", 1, 
                                                              &unibs::deburring::InterpolatorKDL::currentPos_cb, this);
	int counter=0;
  // aspetto finchè non leggo il dato
	while (!m_initial_pos_set && ros::ok())
  { 
    if(counter==5)  //se nessuno pubblica sul topic allora assegno una posizione di default
    {
      m_ptr_nh->getParam("/parameter_set/default_initial_position",tmp);
      Eigen::Array<double,6,1> p = itia::gutils::stdVectorToArray(tmp);
      matrix_frame = itia::gutils::ArrayToMatrix(p);
      m_actual_pos = EigenKDL::frame(matrix_frame);
      m_initial_pos_set = true;
      printf(" [ %s%s:%d%s ]\t %sNo one is publishing in %s'/act_robot_cart_pos'%s topic. Robot initialized with default position. %s \n", 
             YELLOW, __FUNCFILE__, __LINE__, RESET,YELLOW,BLUE,YELLOW,RESET);
      break;
    }  
		printf(" [ %s%s:%d%s ]\t %sWaiting for the robot position from the publisher...%s \n", 
           YELLOW, __FUNCFILE__, __LINE__, RESET,YELLOW,RESET);
		counter++;
    ros::spinOnce(); 
		std::cout.flush();
		sleep(1); 
	}
	m_actual_time = 0.0;
	m_time_offset = 0.0;

	printf ( " [ %s%s:%d%s ]\t Initialization done! \n", GREEN, __FUNCFILE__, __LINE__, RESET);
}

void unibs::deburring::InterpolatorKDL::currentPos_cb(const itia_comau_msgs::CartPnt::ConstPtr& pos)
{	
	Eigen::ArrayXd pos_array(6);
	for(int i=0;i<3;i++)
		pos_array(i) = pos->position.data.at(i)/1000.0;
	for(int i=3;i<6;i++)
		pos_array(i) = pos->position.data.at(i)*PI/180.0;
	
	Eigen::Matrix4d matrix_frame;
	/*KDL::Frame base_to_uframe_kdl;
	matrix_frame = itia::gutils::ArrayToMatrix(m_uframe);
	base_to_uframe_kdl = EigenKDL::frame(matrix_frame);*/
	
	matrix_frame = itia::gutils::ArrayToMatrix(pos_array);
	//m_actual_pos = base_to_uframe_kdl * EigenKDL::frame(matrix_frame);
	m_actual_pos = EigenKDL::frame(matrix_frame);
	
	m_initial_pos_set = true;
	return;
}


bool unibs::deburring::InterpolatorKDL::loadTrj( itia_comau_msgs::trj::Request& req, itia_comau_msgs::trj::Response& res)
{	
	ros::spinOnce();
	if(m_full_trj.Duration()!=0.0)  // nel caso la traiettoria non sia vuota la elimino
  {
		printf( " [ %s%s:%d%s ]\t %sThe trajectory was not empty. The previuos trajectory has been deleted!%s \n", 
            YELLOW, __FUNCFILE__, __LINE__, RESET,YELLOW,RESET);
		m_full_trj.Destroy();
	}
	m_time_offset += m_actual_time;
	
	int npoints=0;
	m_ptr_nh->getParam(req.trj_name.data+"/nPoints",npoints);   
	
	if (npoints<=0)
  {
		printf(" [ %s%s:%d%s ]\t The request trajectory %s%s%s has no points\n",
				   RED, __FUNCFILE__, __LINE__, RESET,BLUE,req.trj_name.data.c_str(),RESET);
		res.res=false;
		return res.res;
	} else 
  {
		printf(" [ %s%s:%d%s ]\t Starting loading trajectory %s%s%s with %s%d%s points \n",
				   GREEN, __FUNCFILE__, __LINE__, RESET,BLUE,req.trj_name.data.c_str(),RESET,BLUE,npoints,RESET);
	}
	
	Eigen::ArrayXXd cart(npoints,6);
	double feedrate;  // linear velocity (m/sec)
	double theta_d_lim; // = 5.0*PI/180.0;  // max allowed angular velocity (rad/sec)
	Eigen::Matrix4d matrix_frame;
	KDL::Frame base_to_uframe_kdl;
	double st[npoints-1];
	
	bool param_found = m_ptr_nh->getParam(req.trj_name.data+"/feedrate", feedrate);
	if(feedrate<=0 || !param_found) feedrate=10.0;  // mm/s
	theta_d_lim = 1.5*feedrate*PI/180.0;  // rad/s
	feedrate = feedrate/1000.0;   // m/s
	
	
	// caricamento posizioni cartesiane Comau (riferite allo userframe)
	itia_comau_msgs::CartPnt pos;
	for(int i=0;i<npoints && ros::ok();i++)
  {
    itia::tutils::getCartPntFromServerParam ( *m_ptr_nh, req.trj_name.data + "/cpoint" + std::to_string(i) +"/", pos);
		cart(i,0) = pos.position.data.at(0)/1000.0;     //m
		cart(i,1) = pos.position.data.at(1)/1000.0;
		cart(i,2) = pos.position.data.at(2)/1000.0;
		cart(i,3) = pos.position.data.at(3)*PI/180.0;   //rad
		cart(i,4) = pos.position.data.at(4)*PI/180.0;
		cart(i,5) = pos.position.data.at(5)*PI/180.0;
		st[i] = pos.exec_time;
	}
	matrix_frame = itia::gutils::ArrayToMatrix(m_uframe);
	base_to_uframe_kdl = EigenKDL::frame(matrix_frame);
	
	
	//Composizione traiettoria KDL (traiettoria del tool riferita alla base del robot, non allo userframe come il vettore c)
	double maxvel = feedrate; 
	double maxacc = maxvel*10.0;
	double eqradius = feedrate/theta_d_lim; 
	KDL::Frame start, end;
	bool is_breaking, is_complete;
	double startvel = 0;
	double endvel = 0;
	int breaking_segments = 0;
	std::vector<double> breaking_startvel;
	breaking_startvel.push_back(0.0);
	
	is_complete = false;
	is_breaking = false;
	std::cout << "maxvel: " << maxvel << " , feedrate: " << feedrate << std::endl << std::endl;

	
	// aggiungo un segmento di raccordo tra la posizione attuale del robot e il primo punto
	KDL::RotationalInterpolation* 	rot_int = new KDL::RotationalInterpolation_SingleAxis();
	KDL::VelocityProfile_Trap* 		leggeMoto = new KDL::VelocityProfile_Trap(maxvel, maxacc);
	matrix_frame = itia::gutils::ArrayToMatrix(cart.row(0));
	end = base_to_uframe_kdl * EigenKDL::frame(matrix_frame);
	KDL::Path*	line = new KDL::Path_Line(m_actual_pos, end, rot_int, eqradius);	
	leggeMoto->SetProfile(0.0, line->PathLength());
	KDL::Trajectory* segmento = new KDL::Trajectory_Segment(line, leggeMoto);
	m_full_trj.Add(segmento);
	std::cout << "Durata raccordo con posizione attuale: " << m_full_trj.Duration() << " s "<< std::endl;


	// controllo quanti segmenti servono per frenare a fine traiettoria
	for(int i=npoints-1; !is_complete;i--)
  {   
		KDL::RotationalInterpolation* rot_int = new KDL::RotationalInterpolation_SingleAxis();
		KDL::VelocityProfile_TrapMerge* leggeMoto = new KDL::VelocityProfile_TrapMerge(maxvel, maxacc, is_breaking);   // essendo l'accelerazione simmetrica, tratto 
                                                                                                                   // i segmenti come se fossero all'inizio
		matrix_frame = itia::gutils::ArrayToMatrix(cart.row(i-1));
		start = base_to_uframe_kdl * EigenKDL::frame(matrix_frame);
		matrix_frame = itia::gutils::ArrayToMatrix(cart.row(i));
		end = base_to_uframe_kdl * EigenKDL::frame(matrix_frame);
		
		KDL::Path*	line = new KDL::Path_Line(start, end, rot_int, eqradius);
		leggeMoto->SetProfile(0.0, line->PathLength(), startvel, endvel, is_complete); 
		
		startvel = leggeMoto->Vel(leggeMoto->Duration());
		breaking_startvel.push_back(startvel);
		breaking_segments++;
	}
	std::cout << "Numero segmenti necessari per frenare: " << breaking_segments << std::endl << std::endl;
	std::cout << "Calcolo LDM e traiettoria" << std::endl << std::endl;
	//generazione traiettoria
	int k=breaking_segments-1;
	for(int i=0;i<(npoints-1);i++)
  {
		if(i<(npoints-1-breaking_segments))
    {
			is_breaking = false;
			startvel = endvel;
			endvel = maxvel;			
		} else 
    {
			is_breaking = true;
			endvel = breaking_startvel.at(k);
			k--;
			startvel = maxvel;
		}
	
		KDL::RotationalInterpolation* 	rot_int = new KDL::RotationalInterpolation_SingleAxis();
		KDL::VelocityProfile_TrapMerge* leggeMoto = new KDL::VelocityProfile_TrapMerge(maxvel, maxacc, is_breaking);

		matrix_frame = itia::gutils::ArrayToMatrix(cart.row(i));
		start = base_to_uframe_kdl * EigenKDL::frame(matrix_frame);
		matrix_frame = itia::gutils::ArrayToMatrix(cart.row(i+1));
		end = base_to_uframe_kdl * EigenKDL::frame(matrix_frame);
		
		KDL::Path*	line = new KDL::Path_Line(start, end, rot_int, eqradius);		
		leggeMoto->SetProfile(0.0, line->PathLength(), startvel, endvel, is_complete);
		KDL::Trajectory* segmento = new KDL::Trajectory_Segment(line, leggeMoto);
		
		std::cout << "Start: " << cart.row(i) << std::endl;
		std::cout << "End: " << cart.row(i+1) << std::endl;
		std::cout << "Lunghezza path: " << line->PathLength() << std::endl;
		std::cout << "Durata segmento: " << segmento->Duration() << std::endl << std::endl;
		m_full_trj.Add(segmento);
	} 
	printf(" [ %s%s:%d%s ]\t %sTraiettoria generata!%s\n",
				 GREEN, __FUNCFILE__, __LINE__, RESET,GREEN,RESET);
	std::cout << "\nDurata triettoria completa: " << m_full_trj.Duration() << std::endl << std::endl;
	
	return res.res=true;
}

bool unibs::deburring::InterpolatorKDL::appendTrj( itia_comau_msgs::trj::Request& req, itia_comau_msgs::trj::Response& res)
{	
	ros::spinOnce();
	if(m_full_trj.Duration()==0.0) //nel caso la traiettoria sia vuota
  {
		printf(" [ %s%s:%d%s ]\t %sThe trajectory is empty. Use the 'loadTrj' service.%s \n", 
           RED, __FUNCFILE__, __LINE__, RESET,RED,RESET);
		return res.res=false;
	}
	
	int npoints=0;
	m_ptr_nh->getParam(req.trj_name.data+"/nPoints",npoints);   
	
	if (npoints<=0)
  {
		printf(" [ %s%s:%d%s ]\t The request trajectory %s%s%s has no points\n",
				   GREEN, __FUNCFILE__, __LINE__, RESET,RED,req.trj_name.data.c_str(),RESET);
		res.res=false;
		return res.res;
	} else 
  {
		printf(" [ %s%s:%d%s ]\t Starting appending trajectory %s%s%s with %s%d%s points \n",
				   GREEN, __FUNCFILE__, __LINE__, RESET,BLUE,req.trj_name.data.c_str(),RESET,BLUE,npoints,RESET);
	}
	
	
	Eigen::ArrayXXd cart(npoints,6);
	double feedrate;  // linear velocity (m/sec)
	double theta_d_lim; // = 5.0*PI/180.0;  // max allowed angular velocity (rad/sec)
	Eigen::Matrix4d matrix_frame;
	KDL::Frame base_to_uframe_kdl;
	double st[npoints-1];
	
	bool param_found = m_ptr_nh->getParam(req.trj_name.data+"/feedrate", feedrate);
	if(feedrate<=0 || !param_found) feedrate=10.0;  // mm/s
	theta_d_lim = 1.5*feedrate*PI/180.0;  // rad/s
	feedrate = feedrate/1000.0;   // m/s
	
	
	// caricamento posizioni cartesiane Comau 
	itia_comau_msgs::CartPnt pos;
	for(int i=0;i<npoints && ros::ok();i++)
  {
    itia::tutils::getCartPntFromServerParam ( *m_ptr_nh, req.trj_name.data + "/cpoint" + std::to_string(i) +"/", pos);
		cart(i,0) = pos.position.data.at(0)/1000.0;     //m
		cart(i,1) = pos.position.data.at(1)/1000.0;
		cart(i,2) = pos.position.data.at(2)/1000.0;
		cart(i,3) = pos.position.data.at(3)*PI/180.0;   //rad
		cart(i,4) = pos.position.data.at(4)*PI/180.0;
		cart(i,5) = pos.position.data.at(5)*PI/180.0;
		st[i] = pos.exec_time;
	}
	matrix_frame = itia::gutils::ArrayToMatrix(m_uframe);
	base_to_uframe_kdl = EigenKDL::frame(matrix_frame);
	
	
	//Composizione traiettoria KDL (traiettoria del tool riferita alla base del robot, non allo userframe come il vettore c)
	double maxvel = feedrate;
	double maxacc = maxvel*10.0;
	double eqradius = feedrate/theta_d_lim; 
	KDL::Frame start, end;
	bool is_breaking, is_complete;
	double startvel = 0;
	double endvel = 0;
	int breaking_segments = 0;
	std::vector<double> breaking_startvel;
	breaking_startvel.push_back(0.0);
	
	is_complete = false;
	is_breaking = false;
	std::cout << "maxvel: " << maxvel << " , feedrate: " << feedrate << std::endl << std::endl;

	//aggiungo un segmento di raccordo
	KDL::RotationalInterpolation* 	rot_int = new KDL::RotationalInterpolation_SingleAxis();
	KDL::VelocityProfile_Trap* 		leggeMoto = new KDL::VelocityProfile_Trap(maxvel, maxacc);
	start = m_full_trj.Pos(m_full_trj.Duration());
	matrix_frame = itia::gutils::ArrayToMatrix(cart.row(0));
	end = base_to_uframe_kdl * EigenKDL::frame(matrix_frame);
	KDL::Path*	line = new KDL::Path_Line(start, end, rot_int, eqradius);
	leggeMoto->SetProfile(0.0, line->PathLength());
	KDL::Trajectory* segmento = new KDL::Trajectory_Segment(line, leggeMoto);
	m_full_trj.Add(segmento);
	std::cout<<"Durata raccordo con ultima posizione traiettoria precedente: "<<segmento->Duration()<<std::endl<<std::endl;
		
	// controllo quanti segmenti servono per frenare a fine traiettoria
	for(int i=npoints-1; !is_complete;i--)
  {   
		KDL::RotationalInterpolation* rot_int = new KDL::RotationalInterpolation_SingleAxis();
		KDL::VelocityProfile_TrapMerge* leggeMoto = new KDL::VelocityProfile_TrapMerge(maxvel, maxacc, is_breaking);   // essendo l'accelerazione simmetrica, tratto 
                                                                                                                   // i segmenti come se fossero all'inizio
    matrix_frame = itia::gutils::ArrayToMatrix(cart.row(i-1));
		start = base_to_uframe_kdl * EigenKDL::frame(matrix_frame);
		matrix_frame = itia::gutils::ArrayToMatrix(cart.row(i));
		end = base_to_uframe_kdl * EigenKDL::frame(matrix_frame);
		KDL::Path*	line = new KDL::Path_Line(start, end, rot_int, eqradius);
		leggeMoto->SetProfile(0.0, line->PathLength(), startvel, endvel, is_complete); 
		
		startvel = leggeMoto->Vel(leggeMoto->Duration());
		breaking_startvel.push_back(startvel);
		breaking_segments++;
	}
	std::cout << "Numero segmenti necessari per frenare: " << breaking_segments << std::endl << std::endl;
	std::cout << "Calcolo LDM e traiettoria" << std::endl << std::endl;
	
	//generazione traiettoria
	int k=breaking_segments-1;
	for(int i=0;i<(npoints-1);i++)
  {
		if(i<(npoints-1-breaking_segments))
    {
			is_breaking = false;
			startvel = endvel;
			endvel = maxvel;			
		} else 
    {
			is_breaking = true;
			endvel = breaking_startvel.at(k);
			k--;
			startvel = maxvel;
		}
	
		KDL::RotationalInterpolation* 	rot_int = new KDL::RotationalInterpolation_SingleAxis();
		KDL::VelocityProfile_TrapMerge* leggeMoto = new KDL::VelocityProfile_TrapMerge(maxvel, maxacc, is_breaking);
		matrix_frame = itia::gutils::ArrayToMatrix(cart.row(i));
		start = base_to_uframe_kdl * EigenKDL::frame(matrix_frame);
		matrix_frame = itia::gutils::ArrayToMatrix(cart.row(i+1));
		end = base_to_uframe_kdl * EigenKDL::frame(matrix_frame);
		KDL::Path*	line = new KDL::Path_Line(start, end, rot_int, eqradius);		
		leggeMoto->SetProfile(0.0, line->PathLength(), startvel, endvel, is_complete);
		KDL::Trajectory* segmento = new KDL::Trajectory_Segment(line, leggeMoto);
		
		std::cout << "Start: " << cart.row(i) << std::endl;
		std::cout << "End: " << cart.row(i+1) << std::endl;
		std::cout << "Lunghezza path: " << line->PathLength() << std::endl;
		std::cout << "Durata segmento: " << segmento->Duration() << std::endl << std::endl;
		m_full_trj.Add(segmento);
	} 
	printf(" [ %s%s:%d%s ]\t %sTraiettoria generata!%s\n",
				 GREEN, __FUNCFILE__, __LINE__, RESET,GREEN,RESET);
	std::cout << "\nDurata triettoria completa: " << m_full_trj.Duration() << std::endl << std::endl;
	
	return res.res=true;
}


bool unibs::deburring::InterpolatorKDL::getPose( itia_motion_msgs::CartPnt_at_Time::Request& req, 
                                                 itia_motion_msgs::CartPnt_at_Time::Response& res )
{	
	if(m_full_trj.Duration()==0.0)
  {
		printf ( " [ %s%s:%d%s ]\t %sThe trajectory is empty!%s \n", RED, __FUNCFILE__, __LINE__, RESET, RED, RESET);
		return false;
	}
	
	if(req.time < m_time_offset) // nel caso la traiettoria sia stata caricata senza esecuzione in atto
		m_time_offset = 0.0;
	
	double time = req.time - m_time_offset;
	std::cout << time << " -> " << m_full_trj.Duration() << std::endl;
	Eigen::Matrix4d matrix_frame;
	Eigen::ArrayXd array(6);
	
	std::string pos_ref = "Robot base frame";
	std::string pos_type = "Euler ZYZ (m,rad)";
	KDL::Frame pos = m_full_trj.Pos(time);
	KDL::Twist vel = m_full_trj.Vel(time);
	KDL::Twist acc = m_full_trj.Acc(time); 
	
	res.point.header.stamp=ros::Time::now();
	res.point.pose_reference = pos_ref;
	res.point.pose_type = pos_type;
	
	res.point.pose.resize(6);
	res.point.velocity.resize(6);
	res.point.acceleration.resize(6);
  matrix_frame = EigenKDL::frame(pos);
  array = itia::gutils::MatrixToArray(matrix_frame);
	for(int i=0;i<6;i++)
  {
		res.point.pose.at(i) = array(i);
		res.point.velocity.at(i) = vel(i);
		res.point.acceleration.at(i) = acc(i);
	}
	
	m_actual_pos = pos;
	m_actual_time = time;
	
	return true;
}



bool unibs::deburring::InterpolatorKDL::reloadReferences(itia_motion_msgs::reloadReferences::Request& req, 
                                                         itia_motion_msgs::reloadReferences::Response& res)
{
	std::vector<double> tmp;
	if(m_ptr_nh->getParam(req.paramNamespace.data+"/bframe",tmp))
		m_bframe = itia::gutils::stdVectorToArray(tmp);
	else
  {
		printf(" [ %s%s:%d%s ]\t %sImpossible to find the %sbase_frame%s definition in the parameter server!%s \n", 
           RED, __FUNCFILE__, __LINE__, RESET,RED,YELLOW,RED,RESET);
    return res.res=false;
  }
	
	if(m_ptr_nh->getParam(req.paramNamespace.data+"/uframe",tmp))
		m_uframe = itia::gutils::stdVectorToArray(tmp);
	else
  {
		printf(" [ %s%s:%d%s ]\t %sImpossible to find the %suser_frame%s definition in the parameter server!%s \n", 
           RED, __FUNCFILE__, __LINE__, RESET,RED,YELLOW,RED,RESET);
    return res.res=false;
  }
	
	if(m_ptr_nh->getParam(req.paramNamespace.data+"/tframe",tmp))
		m_tframe = itia::gutils::stdVectorToArray(tmp);
	else
  {
		printf(" [ %s%s:%d%s ]\t %sImpossible to find the %stool_frame%s definition in the parameter server!%s \n", 
           RED, __FUNCFILE__, __LINE__, RESET,RED,YELLOW,RED,RESET);
    return res.res=false;
  }
	
	if(m_ptr_nh->getParam(req.paramNamespace.data+"/q_avg",tmp))
		m_q_avg = itia::gutils::stdVectorToArray(tmp);
	else 
  {
		printf(" [ %s%s:%d%s ]\t %sImpossible to find the %sq_avg%s definition in the parameter server!%s \n", 
           RED, __FUNCFILE__, __LINE__, RESET,RED,YELLOW,RED,RESET);
    return res.res=false;
  }
	
	Eigen::Matrix4d matrix_frame;
	matrix_frame = itia::gutils::ArrayToMatrix(m_tframe);
	KDL::Frame flange_to_tool_kdl;
	flange_to_tool_kdl = EigenKDL::frame(matrix_frame);
	std::string name = "Tool_frame";
	KDL::Joint j("None");
	KDL::Segment tool(name,j,flange_to_tool_kdl);
	
	m_urdf_chain.segments.back() = tool;
	
	printf ( " [ %s%s:%d%s ]\t Change of references done! \n", GREEN, __FUNCFILE__, __LINE__, RESET);
	
	return res.res=true;
}

