function [cz, cz_z, cz_zz] = limit_penalty(z, z_max, rho_z)
    % c = rho * max(0, z'*z/z_max^2 - 1)^2
    % 要求 z_max > 0，rho >= 0
    g = (z'* z) / z_max^2 - 1;
    cz = rho_z * max(0, g)^2;
    if g > 0
        cz_z = 4 * rho_z * g / z_max^2 * z;
        cz_zz = 4 * rho_z / z_max^2 * (g * eye(length(z)) + z * z' * 2 / z_max^2);
    else
        cz_z = zeros(size(z));  % [2,1]，与 z 同维度
        cz_zz = zeros(length(z));% 2， 得到方阵
    end
end