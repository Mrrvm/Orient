% eR_mbpe_logm = logm(R_pro'*R);
% eR_pro_logm  = logm(R_mbpe'*R);
% 
% q      = rotm2quat(R);
% q_pro  = rotm2quat(R_pro);
% q_mbpe = rotm2quat(R_mbpe);
% eR_pro_q  = 2*acos(quatinv(q_pro)*q');
% eR_mbpe_q = 2*acos(quatinv(q_mbpe)*q');