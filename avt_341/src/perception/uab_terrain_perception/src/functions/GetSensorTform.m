function tf = GetSensorTform(rotation, translation)
    sizeR = length(rotation); 
    if sizeR == 4
        R = quat2rotm(rotation);
    else
        R = rotation;
    end
    tf = rigidtform3d(R, translation);
end