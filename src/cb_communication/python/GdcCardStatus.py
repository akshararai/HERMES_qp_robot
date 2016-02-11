import os,sys


property_list = [##'joint_name',
                 'dof_number', 'card_number',
                 'pos_P_gain', 'pos_I_gain', 'pos_D_gain',
                 'tor_P_gain', 'tor_I_gain', 'tor_D_gain',
                 'valve_manual_cmd', 'valve_dither_amplitude', 'valve_dither_frequency',
                 'valve_DAC_bias',
                 'max_des_pos_limit', 'min_des_pos_limit', 'max_val_cmd_limit', 'min_val_cmd_limit',
                 'load_cell_DAC_bias',
                 'max_position_fault_level', 'min_position_fault_level', 'abs_delta_position_fault_level',
                 'abs_velocity_fault_level', 'abs_position_P_error_fault_level',
                 'abs_torque_fault_level', 'abs_delta_torque_fault_level', 'abs_torque_P_error_fault_level',
                 'communication_fault_level', 'invert_byte', 'mode_byte',
                 'position_P_error', 'position_I_error', 'torque_P_error', 'torque_I_error',
                 'encoder_subfault_mask', 'position_time_stamp',
                 'actual_position', 'actual_velocity', 'actual_torque',
                 'valve_current', 'valve_command',
                 'PID_des_position', 'desired_position', 'PID_des_torque', 'desired_torque',
                 'encoder_value', 'potentiometer_value', 'pwm_current',
                 'status0', 'status1', 'status2', 'status3'
                 ]


class GdcCardStatus:
    def __init__(self):
        self.data = []
        self.names = []
        
    def load(self, filename):
        ins = open( filename, "r" )
        self.data = []
        self.names = []
        for line in ins:
            values = line.split(" ")
            if(len(values) == len(property_list)+1):
                self.names.append(values[0])
                self.data.append(values[1:])

    def save(self, filename):
        ins = open(filename, "w")
        for i in range(0, len(self.names)):
            st = self.names[i] + ' '
            for j in range(0, len(self.data[i])):
                st+= self.data[i][j] + ' '
            st += '\n'
            ins.write(st)
        ins.close()
    