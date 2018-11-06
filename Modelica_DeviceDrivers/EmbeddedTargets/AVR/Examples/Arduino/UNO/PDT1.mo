within Modelica_DeviceDrivers.EmbeddedTargets.AVR.Examples.Arduino.UNO;
block PDT1 "Practical (approximated) PD controller representation"
  extends Modelica.Blocks.Interfaces.SISO;
  parameter Real Kp = 5.5 "Proportional gain";
  parameter Real Td = 0.05 "Derivative time constant";
  parameter Real Nd(min=100*Modelica.Constants.eps) = 10 "The higher Nd, the more ideal the derivative block";
  Real x_scaled;
equation
  der(x_scaled) = -Nd/Td*x_scaled + u;
  y = (-Kp*Nd^2 / Td)*x_scaled + (Kp + Kp*Nd)*u;
  annotation (Icon(graphics={  Text(lineColor = {255, 0, 0}, extent = {{-94, 106}, {90, 28}}, textString = "Kp: %Kp"), Text(lineColor = {255, 0, 0}, extent = {{-92, 38}, {94, -44}}, textString = "Td: %Td"), Text(lineColor = {255, 0, 0}, extent = {{-88, -34}, {88, -98}}, textString = "Nd: %Nd")}, coordinateSystem(initialScale = 0.1)), Diagram(coordinateSystem(initialScale = 0.1)));
end PDT1;
