load('params')



omega_zt = (x_2+x_3);

%% Slip angles
alpha_f = delta_f - (l_1c*x_2+x_1)/v_x;
alpha_r = (l_2c*x_2-x_1)/v_x;
alpha_t = delta_t - (v_yt-l_2t*omega_zt)/v_x;

%% Tire models
Fy_r = 2*k_r*alpha_r;
Fy_f = 2*k_f*alpha_f;
Fy_t = 2*k_t*alpha_t;

%% f1
Fy_hc = (Fy_f*l_1c+delta_M-Fy_r*l_2c-x_2_dot*I_zc)/l3_c;

f1 = -x_1_dot+(Fy_hc+Fy_r+Fy_f)/m_c-v_xc*x_2;


%% f2
Fy_ht = (I_zt*(x_2_dot+x_3_dot)+Fy_t)/l1_t;

v_yt_dot = (Fy_t+Fy_ht)/m_t-v_xt*(omega_zt+x_2);

f2 = -x_1_dot+v_yt_dot+x_3_dot*l_1t+x_2_dot*l_3c+omega_zt*x_1;

%% f3
f3 = Fy_hc+Fy_ht;

%% f4
f4 = -x_4_dot+x_3;