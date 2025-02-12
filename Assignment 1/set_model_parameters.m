function set_model_parameters(ramp_angle, initial_inter_leg_angle, initial_stance_angle)

% Adjust ramp position
offset_pdw = 70;  % default distance in milimeter
z_pdw = offset_pdw-offset_pdw*cosd(initial_stance_angle);
x_pdw = offset_pdw*sind(initial_stance_angle);

offset_ramp = 5;
z_ramp = 500*sind(ramp_angle); 
x_ramp = 500*cosd(ramp_angle);

total_z = -(offset_ramp + offset_pdw + z_ramp - z_pdw);
total_x = x_ramp - x_pdw;
offset_string = "[" + string(total_x) + ",0," + string(total_z) + "]";

% Set parameters
set_param('PDW_Simulation/Ramp Angle','RotationAngle',string(ramp_angle))
set_param('PDW_Simulation/Stance Leg Angle','RyPositionTargetValue',string(initial_stance_angle))
set_param('PDW_Simulation/Inter Leg Angle','PositionTargetValue',string(initial_inter_leg_angle))
set_param('PDW_Simulation/Ramp Angle', 'TranslationCartesianOffset', offset_string)

end

