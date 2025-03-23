g = 9.80665;
L = 0.3;

b_revolute = 0;
b_prismatic = 0;

cover_flage_Ixx = 425.83e-3;
cover_flage_Iyy = 313020.84e-9;
cover_flage_Izz = 171937.07e-9;
cover_flage_mass = 477713.12e-9;

slide_rail_Ixx = 49757667.19e-9;
slide_rail_Iyy = 1225718.53e-9;
slide_rail_Izz = 50950095.13e-9;
slide_rail_mass = 1893.96e-3;

plotter_Ixx = 1020611.12e-9;
plotter_Iyy = 1146620.14e-9;
plotter_Izz = 456002.44e-9;
plotter_mass = 500e-3;

% Ke = 0.5265;
% Km = 0.5265 / 2.0;
% L = 183.67e-6;
% R = 0.6367;
% J = 0.013369679361673;
% B = 0.019688440522932;

%Prismatic Motor
prismatic_ke = 0.1294;
prismatic_km = 0.4123 * prismatic_ke;
prismatic_L = 1.34e-4;
prismatic_R = 0.99;
prismatic_J = 2.3673e-4;
prismatic_B = 0.0088;

prismatic_speed = 500; %mm/s
prismatic_accel = 250; %mm/s^2
prismatic_pulley = 1.5915e-2;
prismatic_max_speed = 400*2*pi*prismatic_pulley/60;

%Revolute Motor
revolute_ke = 0.1696;
revolute_km = 0.4621 * revolute_ke;
revolute_L = 7.356e-4;
revolute_R = 1.3;
revolute_J = 0.0029;
revolute_B = 0.037;

revolute_speed = 1; %rad/s
revolute_accel = 0.4; %rad/s^2

DFFW_t_prismatic = prismatic_L/(prismatic_R*prismatic_pulley);
RFFW_t_prismatic = (plotter_mass*prismatic_pulley*prismatic_pulley*prismatic_R+prismatic_J)/(prismatic_km*prismatic_ke+prismatic_B*prismatic_R);

DFFW_t_revolute = revolute_L/revolute_R;
RFFW_t_revolute = (revolute_R*revolute_J)/(revolute_km*revolute_ke+revolute_B*revolute_R);

v_max = 12.0;

