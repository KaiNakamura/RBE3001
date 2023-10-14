clearvars;
close all;

% Create robot and send it to home position
robot = Robot();
robot.openGripper();
robot.moveToHome(0.5);

% Create camera
camera = Camera();

% Create figure
fig = figure;
ax = axes(fig);
ax.Visible = "off";
fig.Position = [100, 100, 1280, 720];
button = uicontrol(fig, "Style", "PushButton", "String", "Run", "Callback", @handlePress, "UserData", 0);

while true

    while true

        image = camera.getWorkspaceImage();
        [centroids, radii, colors] = ImageProcessing.getCircles(image);
        image = ImageProcessing.drawCircles(image, centroids, radii, colors);
        imshow(image);
        drawnow;

        if button.UserData
            break;
        end

    end

    % Get image of the workspce
    image = camera.getWorkspaceImage();

    % Get circles of the image
    [centroids, radii, colors] = ImageProcessing.getCircles(image);
    image = ImageProcessing.drawCircles(image, centroids, radii, colors);
    imshow(image);

    % Get object points
    objectPoints = camera.imagePointsToObjectPoints(centroids, 5);

    % Pick up object
    for i = 1:height(objectPoints)
        % Define poses
        atObject = [objectPoints(i, :), 90] + [0, 0, 10, 0];
        aboveObject = atObject + [0, 0, 45, 0];
        deliveryZone = colors{i}.deliveryZone;
        aboveDeliveryZone = deliveryZone + [0, 0, 90, -30];

        % Pick and place
        if ~robot.moveToPose(aboveObject, 0.5)
            disp("Could not reach above object");
            continue;
        end

        if ~robot.moveToPose(atObject, 0.3)
            disp("Could not reach object");
            continue;
        end

        robot.closeGripper();
        pause(0.2);
        robot.moveToPose(aboveObject, 0.2);
        robot.moveToPose(aboveDeliveryZone, 0.4);
        robot.moveToPose(deliveryZone, 0.4);
        pause(0.4);
        robot.openGripper();
        pause(0.2);
        robot.moveToPose(aboveDeliveryZone, 0.4);
    end

    % Go back home
    robot.moveToHome(0.5);

    % Reset button
    button.UserData = 0;
end

function handlePress(button, ~)
    button.UserData = 1;
end
