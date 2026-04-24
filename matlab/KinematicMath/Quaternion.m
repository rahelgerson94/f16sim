classdef Quaternion
    % Quaternion utility class (Hamilton convention, scalar-first [q0 q1 q2 q3])
    % Matches the logic of your Python version, including signs and order.

    methods (Static)
        function q = quaternionFromDcm(dcm)
            % dcm: 3x3
            m00 = dcm(1,1); m01 = dcm(1,2); m02 = dcm(1,3);
            m10 = dcm(2,1); m11 = dcm(2,2); m12 = dcm(2,3);
            m20 = dcm(3,1); m21 = dcm(3,2); m22 = dcm(3,3);

            trace = m00 + m11 + m22;

            if trace > 0
                s = 2.0 * sqrt(trace + 1.0);
                w = 0.25 * s;
                x = (m21 - m12) / s;
                y = (m02 - m20) / s;
                z = (m10 - m01) / s;
            elseif (m00 > m11) && (m00 > m22)
                s = 2.0 * sqrt(1.0 + m00 - m11 - m22);
                w = (m21 - m12) / s;
                x = 0.25 * s;
                y = (m01 + m10) / s;
                z = (m02 + m20) / s;
            elseif m11 > m22
                s = 2.0 * sqrt(1.0 + m11 - m00 - m22);
                w = (m02 - m20) / s;
                x = (m01 + m10) / s;
                y = 0.25 * s;
                z = (m12 + m21) / s;
            else
                s = 2.0 * sqrt(1.0 + m22 - m00 - m11);
                w = (m10 - m01) / s;
                x = (m02 + m20) / s;
                y = (m12 + m21) / s;
                z = 0.25 * s;
            end

            q = [w, x, y, z];
            q = q / norm(q);
        end

        function q = quaternionFromEulerAngles321(euler321)
            % euler321 = [phi theta psi] (roll, pitch, yaw), radians
            % Matches your Python signs (note sTheta = -sin(theta/2))
            phi = euler321(1);
            theta = euler321(2);
            psi = euler321(3);

            cPsi   = cos(psi * 0.5);
            sPsi   = sin(psi * 0.5);
            cTheta = cos(theta * 0.5);
            sTheta = -sin(theta * 0.5); % note the minus, as in your Python
            cPhi   = cos(phi * 0.5);
            sPhi   = sin(phi * 0.5);

            q0 = cPhi * cTheta * cPsi + sPhi * sTheta * sPsi;
            q1 = cPhi * sTheta * sPsi  - sPhi * cTheta * cPsi;
            q2 = -cPhi * sTheta * cPsi - sPhi * cTheta * sPsi;
            q3 = sPhi * sTheta * cPsi  - cPhi * cTheta * sPsi;
            q  = [q0, q1, q2, q3];
            q  = q / norm(q);
        end

        function qp = multiply(q, p)
            % q, p: 1x4 (scalar-first)
            q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
            p0 = p(1); p1 = p(2); p2 = p(3); p3 = p(4);

            w = q0*p0 - q1*p1 - q2*p2 - q3*p3;
            x = q0*p1 + q1*p0 + q2*p3 - q3*p2;
            y = q0*p2 - q1*p3 + q2*p0 + q3*p1;
            z = q0*p3 + q1*p2 - q2*p1 + q3*p0;
            qp = [w, x, y, z];
        end

        function qc = conjugate(q)
            qc = [q(1), -q(2), -q(3), -q(4)];
        end

        function vRot = rotateVectorByQuaternion(dcmOrQ, v)
            % v: 1x3
            % dcmOrQ: either 3x3 DCM or 1x4 quaternion
            if isequal(size(dcmOrQ), [3,3])
                q = Quaternion.quaternionFromDcm(dcmOrQ);
            elseif isequal(size(dcmOrQ), [1,4]) || isequal(size(dcmOrQ), [4,1])
                q = dcmOrQ(:).'; % row
            else
                error('rotateVectorByQuaternion: bad input size. Use 3x3 DCM or 1x4 quaternion.');
            end

            q = q / norm(q);
            vQuat = [0, v(:).'];           % [0 vx vy vz]
            qConj = Quaternion.conjugate(q);

            qv = Quaternion.multiply(q, vQuat);
            rotatedQuat = Quaternion.multiply(qv, qConj);
            vRot = rotatedQuat(2:4).'; % return 3x1 column vector
        end

        function R = dcmFromEulerAngles32(angles32)
            % angles32 = [phi theta psi], but implements your exact matrix
            phi   = angles32(1);
            theta = angles32(2);
            psi   = angles32(3);

            cpsi   = cos(psi);   spsi   = sin(psi);
            ctheta = cos(theta); stheta = sin(theta);

            % [IB] = [Rx]*[Ry]*[Rz] (as in your comment)
            R = [ cpsi*ctheta,     spsi*ctheta,     stheta; ...
                 -spsi,            cpsi,            0;      ...
                 -cpsi*stheta,    -spsi*stheta,     ctheta ];
        end

        function R = dcmFromEulerAngles321(angles321)
            % 3-2-1 (yaw-pitch-roll) sequence, exact to your Python code & signs
            phi   = angles321(1);
            theta = angles321(2);
            psi   = angles321(3);

            cphi   = cos(phi);   sphi   = sin(phi);
            ctheta = cos(theta); stheta = sin(theta);
            cpsi   = cos(psi);   spsi   = sin(psi);

            R = [ cpsi*ctheta,                        spsi*ctheta,                       stheta; ...
                 -cpsi*stheta*sphi - spsi*cphi,      -spsi*stheta*sphi + cpsi*cphi,     ctheta*sphi; ...
                 -cpsi*stheta*cphi + spsi*sphi,      -spsi*stheta*cphi - cpsi*sphi,     ctheta*cphi ];
        end

        function e321 = eulerAngles321FromQuaternion(q)
            % "formulas from Stevens", with conjugate first (matches Python)
            q = q(:).';                 % row
            q = Quaternion.conjugate(q);
            if vecnorm(q)  <= 0.01
                %in case the user passes a quaternion 
                %derivatve qDot = [0,0,0,0],
                % return zero euler rates.
                % without this guard, will retrun NaNs
                % note, qDot will equal 0 if
                % omega = 0
                e321 = [0,0,0]';
                return;
            else

            if abs(norm(q) - 1) > 1e-8
                warning('norm(q) ~= 1, normalizing...');
                q = q / norm(q);
            end
                q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
                q0_2 = q0*q0; q1_2 = q1*q1; q2_2 = q2*q2; q3_2 = q3*q3;
                phi   = atan2( 2*(q2*q3 + q0*q1),  q0_2 - q1_2 - q2_2 + q3_2 );
                theta = asin ( 2*(q1*q3 - q0*q2) );
                psi   = atan2( 2*(q1*q2 + q0*q3),  q0_2 + q1_2 - q2_2 - q3_2 );
    
                e321 = [phi, theta, psi]'; %return a 3x1 column vector 
            end
        end

        function e321 = eulerAngles321FromQuaternion2(q)
            % "formulas from Stevens", transposed DCM variant (matches Python)
            q = q(:).';
            if abs(norm(q) - 1) > 1e-12
                warning('norm(q) ~= 1, normalizing...');
                q = q / norm(q);
            end
            q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

            q0_2 = q0*q0; q1_2 = q1*q1; q2_2 = q2*q2; q3_2 = q3*q3;

            phi   = atan2( 2*(q2*q3 - q0*q1),  q0_2 - q1_2 - q2_2 + q3_2 );
            theta = asin ( 2*(q1*q3 - q0*q2) );
            psi   = atan2( 2*(q1*q2 - q0*q3),  q0_2 + q1_2 - q2_2 - q3_2 );

            e321 = [phi, theta, psi];
        end

        function dq = computeDerivative(q, w)
            % q: 1x4, w: 1x3 angular velocity
            wq = Quaternion.getPureQuaternion(w);
            dq = 0.5 * Quaternion.multiply(q, wq);
        end

        function qv = getPureQuaternion(v)
            v = v(:).';
            qv = [0, v];
        end

        function e321 = eulerAngles321FromDcm(dcm)
            % Schaub eq. 3.34(a)-(c) (as in your Python comment)
            theta = asin(dcm(1,3));
            psi   = atan2(dcm(1,2), dcm(1,1));
            phi   = atan2(dcm(2,3), dcm(3,3));
            e321  = [phi, theta, psi];
        end

        function R = quat2Dcm(q)
            % scalar-first Hamilton, matches your Python matrix
            q = q(:).';
            q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

            R = [ 1 - 2*(q2^2 + q3^2),     2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2); ...
                  2*(q1*q2 + q0*q3),       1 - 2*(q1^2 + q3^2),   2*(q2*q3 - q0*q1); ...
                  2*(q1*q3 - q0*q2),       2*(q2*q3 + q0*q1),     1 - 2*(q1^2 + q2^2) ];
        end
    end
end
