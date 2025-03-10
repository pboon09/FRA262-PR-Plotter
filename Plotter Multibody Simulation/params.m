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

Ke = 0.5265;
Km = 0.5265 / 2.0;
L = 183.67e-6;
R = 0.6367;
J = 0.013369679361673;
B = 0.019688440522932;

V_max = 12.0;
Prisnmatic_Speed_Max = 500e-3;

Prismatic_pulley = 1.5915e-2;

DFFW_t_prismatic = L/(R*Prismatic_pulley);
RFFW_t_prismatic = (plotter_mass*Prismatic_pulley*Prismatic_pulley*R+J)/(Km*Ke+B*R);

DFFW_t_revolute = L/R;
RFFW_t_revolute = (R*J)/(Km*Ke+B*R);
