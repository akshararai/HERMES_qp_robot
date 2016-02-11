/* Trajectory tracking
Inputs desired joint
Reads a file with desired joint angle, velocity and tracks it
author: Akshara Rai
date: Jan 2015
*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "utility.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"
#include "SL_dynamics.h"
#include "SL_sensor_proc.h"
#include "SL_filters.h"


double tmp0;
double tmp1;
double tmp2;  

//#include <iostream>
/* local variables */
static SL_DJstate *target;
static SL_Jstate *joint_raw;
static double           *misc_raw_sensor;

static double     task_time;
static fMatrix    traj_pos=NULL; 
static fMatrix    traj_vel=NULL;
static fMatrix    traj_acc=NULL;
static fMatrix    traj_uff=NULL;
static fMatrix    traj_vff=NULL;  
static int        *column_map;
static Filter    *fth;
static Filter    *fthd;
static Filter    *fthdd;
static Filter *fmisc_sensor;


char       string[100];

//Predefine the size? Or input?

static int        n_rows=0,n_cols=0, n_vars=0;

// Which joint would it be?
static int	  targetDOF=0;
static double     sampling_freq;
static int        traj_index;
static int        repeat_flag;
static int        invdyn_flag=FALSE;

/* global functions */

void add_traj_track_task(void);

/* local functions */
static int init_traj_track_task(void);
static int run_traj_track_task(void);
static int change_traj_track_task(void);
static int read_traj_file(int flag);
static int check_traj_range(void);
 

// Add task

void add_traj_track_task( void )

{
  int i, j;
  static int firsttime = TRUE;
  
  if (firsttime) {
    firsttime = FALSE;
    
    target     = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate), MY_STOP);
    joint_raw     = (SL_Jstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate), MY_STOP);
    column_map = my_ivector(1,n_dofs);
    fth    = (Filter *)my_calloc(n_dofs+1,sizeof(Filter),MY_STOP);
    fthd   = (Filter *)my_calloc(n_dofs+1,sizeof(Filter),MY_STOP);
    fthdd  = (Filter *)my_calloc(n_dofs+1,sizeof(Filter),MY_STOP);


 for (i=1; i<=n_dofs; ++i) {
    sprintf(string,"%s_rth",joint_names[i]);
    addVarToCollect((char *)&(joint_raw[i].th),string,"rad", DOUBLE,FALSE);
    sprintf(string,"%s_rthd",joint_names[i]);
    addVarToCollect((char *)&(joint_raw[i].thd),string,"rad/s", DOUBLE,FALSE);
    sprintf(string,"%s_rload",joint_names[i]);
    addVarToCollect((char *)&(joint_raw[i].load),string,"Nm", DOUBLE,FALSE);
  }


  for (i=1; i<=n_misc_sensors; ++i) {
    sprintf(string,"%s_r",misc_sensor_names[i]);
    addVarToCollect((char *)&(misc_raw_sensor[i]),string,"-", DOUBLE,FALSE);
  }

    
    addTask("Traj tracking Task", init_traj_track_task, 
	    run_traj_track_task, change_traj_track_task);
  }
  
}    



// Init task


static int init_traj_track_task(void)
{
  int j, i, c;
  char string[100];
  double max_range=0;
  static int ans = 0;

  /* check whether any other task is running */
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("New task can only be run if no other task is running!\n");
    return FALSE;
  }

  printf("Filter initializing");
    // initialize the filters
    init_filters();
    for (i=1; i<=n_dofs; ++i) {
      fth[i].cutoff = 3.5;
      fthd[i].cutoff = 3.5;
      fthdd[i].cutoff = 3.5;
    }

  /* zero the filters */
  for (i=1; i<=n_dofs; ++i) { 
   for (j=0; j<=FILTER_ORDER; ++j) {
      fth[i].raw[j] = fth[i].filt[j] = 0;
      fthd[i].raw[j] = fthd[i].filt[j] = 0;
      fthdd[i].raw[j] = fthdd[i].filt[j] = 0;
    }
  }
  printf("Initialized");


  /* use desired or real kinematic data? */
  //get_int("Extract from DESIRED data? Yes=1 No=0",ans,&ans);

  /* read the script for this task */
  if (!read_traj_file(1))
    return FALSE;

  if (!check_traj_range())
    return FALSE;
  
  /* check for repeatable task : first and last entries same? */ 
  repeat_flag = TRUE;
  for (i=1;i<=n_vars;++i){
    if((traj_pos[1][i]==traj_pos[n_rows][i]) && 
       (traj_vel[1][i]==traj_vel[n_rows][i]) && 
       (traj_uff[1][i]==traj_uff[n_rows][i])&&
			 (traj_vff[1][i]==traj_vff[n_rows][i])){
      ;
    }else 
      repeat_flag = FALSE;
  }
  /* do we want to repeat task */
  /*if (repeat_flag){
    ans = 1;
    get_int("Do you want this task to repeat itself? 1=y,0=n",ans,&ans);
    if (ans!=1)
      repeat_flag = FALSE;
  }*/


  /* enable inverse dynamics control */
  get_int("Use inverse dynamics",invdyn_flag,&invdyn_flag);

  /* go to start posture */
  traj_index = 1;
  bzero((char *)&(target[1]),n_dofs*sizeof(target[1]));
  for (i=1; i<=n_dofs; ++i) {
    target[i] = joint_des_state[i];
    if (column_map[i] != 0)
      target[i].th = traj_pos[ traj_index ][ column_map[i] ];
			target[i].thd = traj_vel[ traj_index ][ column_map[i] ];
			target[i].uff = traj_uff[ traj_index ][ column_map[i] ];
			target[i].vff = traj_vff[ traj_index ][ column_map[i] ];
  }

  if (invdyn_flag) {
    if (!go_target_wait_ID(target))
      return FALSE;
  } else {
    if (!go_target_wait(target))
      return FALSE;
  }
  /*int tmp = n_rows/2;
  printf("idx %d", tmp);
  for (i=1; i<=n_dofs; ++i) {                                                                                 
    target[i] = joint_des_state[i];                                                                           
    if (i == 18){  // only for knee - to remove backlash
			if(traj_vel[tmp][column_map[i]] > 0)                                                                                    
      target[i].th -= 0.1;
      else
			target[i].th +=0.1;   
    }                                            
  }                                                                                                           
   if (invdyn_flag) {                                                                                          
    if (!go_target_wait_ID(target))                                                                           
      return FALSE;                                                                                           
    } else {                                                                                                    
    if (!go_target_wait(target))                                                                              
      return FALSE;                                                                                           
  }                                                                                                           
  

  for (i=1; i<=n_dofs; ++i) {                                                                                 
    target[i] = joint_des_state[i];                                                                           
    if (i == 18)  { // only for knee                                                                                 
      if(traj_vel[tmp][column_map[i]] > 0)                                                                     
      target[i].th += 0.1;                                                                                    
      else                                                                                                    
      target[i].th -=0.1;     
    }                                                                                
  } */                                                                                                          
   if (invdyn_flag) {                                                                                         
    if (!go_target_wait_ID(target))                                                                           
      return FALSE;                                                                                           
  } else  {                                                                                                    
    if (!go_target_wait(target))                                                                              
      return FALSE;                                                                                           
  }                                                                                                           
 
  /* do we really want to do this task? */


  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  
  if (ans != 1) 
    return FALSE;

  task_time = 0.0;
  scd();

  return TRUE;

}


// Running the task

static int run_traj_track_task(void)
{
  int j, i;
  double time_of_traj_index;
  double time_of_next_traj_index;
  double aux;
    
  task_time += 1./(double)task_servo_rate;
  
  time_of_traj_index      = ((double)(traj_index-1))/sampling_freq;
  time_of_next_traj_index = ((double)(traj_index))/sampling_freq;

  /* the statement below takes care of numerical round problems */
  while (task_time - time_of_next_traj_index >= -0.00001) {
    if (++traj_index >= n_rows) {
      if(repeat_flag){
	traj_index=1;
	task_time = 1./(double)task_servo_rate;
      } else{
	setTaskByName(NO_TASK);
	return TRUE;
      }
    }
    time_of_traj_index      = (double)(traj_index-1)/sampling_freq;
    time_of_next_traj_index = (double)(traj_index)/sampling_freq;
  } 

// column map defines the joints which are tracking.
//  travel in task time from position to position							  
  joint_raw = joint_state;

  for (i=1; i<=n_dofs; ++i) {
    if (column_map[i] != 0) {

      joint_des_state[i].th  = traj_pos[ traj_index ][ column_map[i] ] +
	(traj_pos[ traj_index+1 ][ column_map[i] ]-traj_pos[ traj_index][ column_map[i]]) * 
	sampling_freq * (task_time - time_of_traj_index);
			

      joint_des_state[i].thd = traj_vel[ traj_index ][ column_map[i] ] +
	(traj_vel[ traj_index+1 ][ column_map[i]]-traj_vel[ traj_index ][ column_map[i]]) * 
	sampling_freq * (task_time - time_of_traj_index);


      joint_des_state[i].thdd = traj_acc[ traj_index ][ column_map[i] ] +
	(traj_acc[ traj_index+1 ][ column_map[i]]-traj_acc[ traj_index ][ column_map[i]]) * 
	sampling_freq * (task_time - time_of_traj_index);


      joint_des_state[i].uff  = traj_uff[ traj_index ][ column_map[i] ] +
	(traj_uff[ traj_index+1 ][ column_map[i]]-traj_uff[ traj_index ][ column_map[i]]) * 
	sampling_freq * (task_time - time_of_traj_index);

      joint_des_state[i].vff  = traj_vff[ traj_index ][ column_map[i] ] +                                     
  (traj_vff[ traj_index+1 ][ column_map[i]]-traj_vff[ traj_index ][ column_map[i]]) *                         
  sampling_freq * (task_time - time_of_traj_index);                                 
                          
			//if(i==18) printf("feedforward valve, %f \n", joint_des_state[i].vff);
    }
  }

  
  /*for (i=1; i<=n_dofs; ++i) {
    aux = (joint_raw[i].th-joint_des_state[i].th)*(double)task_servo_rate;
    target[i].thd  = filt(aux,&(fthd[i]));
    aux = (joint_raw[i].thd-joint_des_state[i].thd)*(double)task_servo_rate;
    target[i].thdd  = filt(aux,&(fthd[i]));
  }
  joint_des_state[i].thd  = target[i].thd;
  joint_des_state[i].thdd = target[i].thdd;
  */
  /*joint_raw = joint_state;

  for (i=1; i<=n_dofs; ++i) {
    joint_state[i].th    = filt(joint_raw[i].th,&fthd[i]);
    joint_state[i].thd   = filt(joint_raw[i].thd,&fthd[i]);
    aux = (fthd[i].raw[1]-fthd[i].raw[2])*(double)task_servo_rate;
    joint_state[i].thdd  = filt(aux,&fthd[i]);
  }*/



  //if (invdyn_flag)
    //SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);

  //read_user_sensors(joint_raw,misc_raw_sensor) ;
  /*joint_raw = joint_state;

  for (i=1; i<=n_dofs; ++i) {
    joint_state[i].th    = filt(joint_raw[i].th,&fth[i]);
    printf("raw = %f, filtered = %f \n", joint_raw[i].th, joint_state[i].th);
    joint_state[i].thd   = filt(joint_raw[i].thd,&fthd[i]);
    aux = (fthd[i].raw[1]-fthd[i].raw[2])*(double)task_servo_rate;
    joint_state[i].thdd  = filt(aux,&fthdd[i]);
  }*/

  return TRUE;

}

static int change_traj_track_task(void)
{

  return TRUE;

}


// Read trajectory file


static int 
read_traj_file(int flag)
{
  int j,i,r,k,rc;
  static char string[100];
  static char fname[100] = "track_hfe.txt";
  static char current_fname[100] = "";
  FILE * fp=NULL;
  int    found = FALSE;
  char **vnames;
  char **units;
  int    buffer_size;
  fMatrix buff;
  int    aux;
  int    ans=0;

  /* open the file, and parse the parameters */

  while (TRUE) {
    if (!get_string("Name of the Traj File\0",fname,fname)) 
      return FALSE;

    if (strcmp(fname,current_fname) ==0) {
      get_int("Re-read the trajectory file?",ans,&ans);
    }
    
    if (strcmp(fname,current_fname) !=0 || ans) {
      /* try to read this file */
      sprintf(string,"%s%s",PREFS,fname);
      fp = fopen(string,"r");
      if (fp != NULL) {
        break;
      } else {
        printf("ERROR: Could not open file >%s<\n",string);
      }
    } else {
      found = TRUE;
      break;
    }

  }

  if (strcmp(fname,current_fname) !=0 || ans) {
    
    /* clear the current parameters */
    if (traj_pos != NULL) {
      my_free_fmatrix(traj_pos,1,n_rows,1,n_vars);
      traj_pos = NULL;
    }
    if (traj_vel != NULL) {
      my_free_fmatrix(traj_vel,1,n_rows,1,n_vars);
      traj_vel = NULL;
    } 
    if (traj_acc != NULL) {
      my_free_fmatrix(traj_acc,1,n_rows,1,n_vars);
      traj_acc = NULL;
    }
    if (traj_uff != NULL) {
      my_free_fmatrix(traj_uff,1,n_rows,1,n_vars);
      traj_uff = NULL;
    }
    if (traj_vff != NULL) {                                                                                   
      my_free_fmatrix(traj_vff,1,n_rows,1,n_vars);                                                            
      traj_vff = NULL;                                                                                        
    }                                                                                                         
  
    /* get the number of rows, columns, sampling frequency
       and calc the bufer_size */
    rc=fscanf(fp,"%d %d %d %lf",&buffer_size, &n_rows,&n_cols,&sampling_freq);
    printf("%d %d \n", n_rows, n_cols);
    /* alocate memory for the variable names and units
       use MY_STOP for checking for errors in allocation */
    vnames = (char **)my_calloc(n_cols+1, sizeof(char *), MY_STOP);
    units = (char **)my_calloc(n_cols+1, sizeof(char *), MY_STOP);
    
    for (i=1;i<=n_cols;++i){
      vnames[i] = (char *) my_calloc(40, sizeof(char), MY_STOP);
      //units[i] = (char *) my_calloc(40, sizeof(char), MY_STOP);
      rc=fscanf(fp, "%s ", vnames[i]);
    }
    
  printf("Reached 1\n"); 
   /* there are two extra blank chrs at the end of the block
       and a line return which we must account for */
    
    /* read file into a buffer and check if the matrix size is correct */  
  buff = my_fmatrix(1,n_rows,1,n_cols);
  printf("Reached 2 \n");  
    
  //fpos_t pos;
  //fgetpos(fp, &pos);
  for (j=1; j<=n_rows; ++j) {
    for (i=1; i<=n_cols; ++i) {
      //fsetpos(fp, &pos);
      fscanf(fp, "%f", &buff[j][i]);
      //fgetpos(fp, &pos);
      printf("buff[%d][%d]:%f", j,i,buff[j][i]);
    }
  }
  printf("Reached 3\n");
  fclose(fp);
    
    /* initialize vars (number of joint names used in file)  
       and look for  the joint names present in file*/
    n_vars = 0;
    for (i=1;i<=n_dofs;++i) {
      if (flag)
	sprintf(string, "%s_des_th",joint_names[i]);
      else
	sprintf(string, "%s_th",joint_names[i]);
      for (j=1;j<=n_cols;++j){
	if (strcmp(string,vnames[j])==0){
	  ++n_vars;
	  break;
	}
      }
    }
    /* create the pos, vel, acc , uff matrices that define the trajectory */
    traj_pos = my_fmatrix(1,n_rows,1,n_vars);
    traj_vel = my_fmatrix(1,n_rows,1,n_vars);
    traj_acc = my_fmatrix(1,n_rows,1,n_vars);
    traj_uff = my_fmatrix(1,n_rows,1,n_vars);
    traj_vff = my_fmatrix(1,n_rows,1,n_vars);  
    
    /* initialize column_map , routine to fill pos matrix */
    n_vars = 0;
    for (i=1;i<=n_dofs;++i) {
      if (flag)
	sprintf(string, "%s_des_th",joint_names[i]);
      else
	sprintf(string, "%s_th",joint_names[i]);
      column_map[i] = 0;
      
      for (j=1;j<=n_cols;++j){
	if (strcmp(string,vnames[j])==0){
	  ++n_vars;
	  
	  found = TRUE;
	  /*map the used column number into the column map with N_DOF+1 columns*/
	  column_map[i] = n_vars;
	  
	  /* fill the pos matrix using the right column from buffer*/
	  for (r=1;r<=n_rows;++r){
	    traj_pos[r][n_vars] = buff[r][j];
	    printf("pos: %f \n", traj_pos[r][n_vars]);
	  }
	  
	  /* also check for velocity, acceleration, and uff information
	     use the same value of n-vars to fill the remaining matrices*/
	  printf("reached 4 \n");
	  if (flag)
	    sprintf(string, "%s_des_thd",joint_names[i]);
	  else
	    sprintf(string, "%s_thd",joint_names[i]);

	  for (k=1;k<=n_cols;++k){
	    if (strcmp(string,vnames[k])==0){
	      for (r=1;r<=n_rows;++r){
		traj_vel[r][n_vars] = buff[r][k];
		printf("vel: %f \n", traj_vel[r][n_vars]);
	      }
	    }
	  }
	  
	  if (flag)
	    sprintf(string, "%s_des_thdd",joint_names[i]);
	  else
	    sprintf(string, "%s_thdd",joint_names[i]);

	  for (k=1;k<=n_cols;++k){
	    if (strcmp(string,vnames[k])==0){
	      for (r=1;r<=n_rows;++r){
		traj_acc[r][n_vars] = buff[r][k];
		printf("acc: %f \n", traj_acc[r][n_vars]);
	      }
	    }
	  }
	  
	  sprintf(string, "%s_uff",joint_names[i]);
	  for (k=1;k<=n_cols;++k){
	    if (strcmp(string,vnames[k])==0){
	      for (r=1;r<=n_rows;++r){
		traj_uff[r][n_vars] = buff[r][k];
		printf("uff: %f \n", traj_uff[r][n_vars]);
	      }
	    }
	  }

    sprintf(string, "%s_vff",joint_names[i]);                                                                 
    for (k=1;k<=n_cols;++k){                                                                                  
      if (strcmp(string,vnames[k])==0){                                                                       
        for (r=1;r<=n_rows;++r){                                                                              
    traj_vff[r][n_vars] = buff[r][k];                                                                         
    printf("vff: %f \n", traj_vff[r][n_vars]);                                                                
        }                                                                                                     
      }                                                                                                       
    }                                                                                                         
 
	  /* assume only one variable of each kind exists*/
	  break;
	}
      }
    }
    
    /* free up memory by deallocating resources */
    my_free_fmatrix (buff,1,n_rows,1,n_cols);
    for (i=1;i<=n_cols;++i){
      free(vnames[i] ); 
      free(units[i]) ;
    }
    
    free(units);
    free(vnames);

  }

  /* keep track fo the current trajectory file to speed up reading */
  strcpy(current_fname,fname);

  return found;

}



/*!*****************************************************************************
 *******************************************************************************
  \note  check_traj_range
  \date  June 1999

  \remarks 

  the given traj lies within joint limits or not

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
check_traj_range(void)
{
  int i,j;
  int idno;
  
  for(i=1;i<=n_dofs;++i){
    if (column_map[i] != 0) {
      idno = column_map[i];
      for(j=1;j<=n_rows;++j){
	if ((traj_pos[j][idno] > joint_range[i][MAX_THETA]) ||
	    ( traj_pos[j][idno]< joint_range[i][MIN_THETA])){
	  printf("Joint Angle Limits Exceeded in joint %s (is %1.3f, should be in range [%1.3f-%1.3f]) at time %f (tick %d)",joint_names[i],traj_pos[j][idno],joint_range[i][MIN_THETA],joint_range[i][MAX_THETA],((double) j)/sampling_freq,j);
	  return FALSE;
	}
	if ((traj_uff[j][idno] > u_max[i]) ||
	    ( traj_uff[j][idno]< -u_max[i])){
	  printf("Feedforward Commands Exceeded in %s a(is %1.3f, should be in range [%1.3f-%1.3f]) at time %f (tick %d)",joint_names[i],traj_uff[j][idno],-u_max[i],u_max[i],((double) j)/sampling_freq,j);
	  return FALSE;
	}
      }
    }
  }

  return TRUE;
}





















































