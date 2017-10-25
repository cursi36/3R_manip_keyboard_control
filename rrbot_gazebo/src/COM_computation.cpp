#include "keyboard_control.h"


KDL::Vector keyboard_control::KeyboardControl::COM_computation (KDL::JntArray &q) 
{
// ======================= Take segments and their inertia parameters ======================= //
int iii = 0;
		Links_Pose_World = KDL::Frame(KDL::Vector(0, 0, 0));
		double Mass = 0;
		KDL::Vector COG_tot;

for (int ii = 0; ii < nseg; ii++)
{

Links_Inertia[ii] = (Segments[ii].getInertia());	
COG[ii] = (Links_Inertia[ii].getCOG()); //In Link's root_frame
double mass = Links_Inertia[ii].getMass();
Mass += mass;

///////////////////
/* Check joints */
joint_type = (Segments[ii].getJoint().getTypeName());


///F_referce i+1 is F_ tip i

/* F_tip i wrt F_root_i = F_tip_i-1*/
if ( joint_type != "None")
{
Links_Pose = Segments[ii].pose(q(iii));
 iii += 1;
}
else {
Links_Pose = Segments[ii].pose(0);
}

Links_Pose_World = Links_Pose_World*Links_Pose;



// COG of each link WRT world frame
COG[ii] = Links_Pose_World*COG[ii];

/*ROS_INFO_STREAM(ii);
ROS_INFO_STREAM(mass);*/

COG_tot += mass*COG[ii];
//endfor
}

COG_tot = COG_tot/Mass;

//ROS_INFO_STREAM(COG_tot);

return COG_tot;
}
