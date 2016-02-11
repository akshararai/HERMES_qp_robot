/*!
 * \file SL_user_fake.h
 *
 * \brief this file contains definitions that are normally to be found in cb/SL_user.h, to be removed later
 *
 *       \date Aug 14, 2009
 *     \author jbuchli
 */

#ifndef SL_USER_FAKE_H_
#define SL_USER_FAKE_H_


/* define the DOFs of this robot */
//be careful the mouth is missing
enum RobotDOFs {
  BASE=0,


  N_ROBOT_DOFS
};



/* number of degrees-of-freedom of robot */
#define N_DOFS (N_ROBOT_DOFS-1)

#endif /* SL_USER_FAKE_H_ */
