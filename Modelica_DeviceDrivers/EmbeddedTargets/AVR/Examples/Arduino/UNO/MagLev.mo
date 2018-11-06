within Modelica_DeviceDrivers.EmbeddedTargets.AVR.Examples.Arduino.UNO;
model MagLev "Magnetic Levitation"
  import MagLev;
  extends .Modelica.Icons.Example;
  import Modelica_DeviceDrivers.EmbeddedTargets.AVR;
  constant Real coilVSource = 1.3 "hack using the 5V Power Supply Module";
  inner Modelica_DeviceDrivers.EmbeddedTargets.AVR.Blocks.Microcontroller mcu(desiredPeriod = 0.01, platform = Modelica_DeviceDrivers.EmbeddedTargets.AVR.Types.Platform.ATmega328P)
  annotation(Placement(visible = true, transformation(origin = {-67, 67}, extent = {{-23, -23}, {23, 23}}, rotation = 0)));
  Modelica_DeviceDrivers.EmbeddedTargets.AVR.Blocks.SynchronizeRealtime synchronizeRealtime1(timer = Modelica_DeviceDrivers.EmbeddedTargets.AVR.Types.TimerSelect.Timer0)  annotation(Placement(visible = true, transformation(origin = {-4, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Blocks.DigitalWriteBoolean writeBool(pin = Modelica_DeviceDrivers.EmbeddedTargets.AVR.Types.Pin.'5', port = Modelica_DeviceDrivers.EmbeddedTargets.AVR.Types.Port.B)  annotation(Placement(visible = true, transformation(origin={70,30},   extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y = mod(time, 1) >= 0.5)  annotation(Placement(visible = true, transformation(origin={20,30},    extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AVR.Blocks.ADC adc_e(
    voltageReference=5,
    voltageReferenceSelect=Modelica_DeviceDrivers.EmbeddedTargets.AVR.Types.VRefSelect.AREF,
    analogPort=Modelica_DeviceDrivers.EmbeddedTargets.AVR.Types.AnalogPort.A1)
    "Hall sensor voltage"
    annotation (Placement(transformation(extent={{-100,-60},{-80,-40}})));

  Modelica_DeviceDrivers.EmbeddedTargets.AVR.Blocks.PWM pwm(
    prescaler=Modelica_DeviceDrivers.EmbeddedTargets.AVR.Types.TimerPrescaler.
        '1/1024',
    timer=Modelica_DeviceDrivers.EmbeddedTargets.AVR.Types.TimerSelect.Timer1,
    timerNumbers={Modelica_DeviceDrivers.EmbeddedTargets.AVR.Types.TimerNumber.A})
                                                                                                                                                                                                            annotation (
  Placement(visible = true, transformation(origin={90,-50},   extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AVR.Examples.Arduino.UNO.PDT1
                         pDT1(
    Kp=15,
    Td=0.05,
    Nd=5) annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
  Modelica.Blocks.Math.Feedback feedback
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Modelica.Blocks.Sources.RealExpression d_deviate(y=0) annotation (Placement(
        visible=true, transformation(
        origin={-90,0},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  Modelica.Blocks.Sources.RealExpression e_e(y=2.791198)
    annotation (Placement(transformation(extent={{-100,-38},{-80,-18}})));
  Modelica.Blocks.Math.Add sub1(k2=-1)
    annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
  Modelica.Blocks.Sources.RealExpression v_e(y=0.659957)
                                                     annotation (Placement(
        visible=true, transformation(
        origin={-10,-20},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  Modelica.Blocks.Math.Add add1(k2=+1)
    annotation (Placement(transformation(extent={{20,-20},{40,0}})));
  Modelica.Blocks.Math.Gain gain(k=(1/coilVSource)*255)
    annotation (Placement(transformation(extent={{60,-20},{80,0}})));
  Modelica.Blocks.Math.RealToInteger realToInteger
    annotation (Placement(transformation(extent={{40,-60},{60,-40}})));
equation
  connect(booleanExpression1.y, writeBool.u) annotation(Line(points={{31,30},{58,
          30}},                                                                            color = {255, 0, 255}));
             /* synchronizeRealtime1.actualInterval is not legal in experiment annotation*/
  connect(feedback.y, pDT1.u)
    annotation (Line(points={{-41,0},{-22,0}}, color={0,0,127}));
  connect(d_deviate.y, feedback.u1)
    annotation (Line(points={{-79,0},{-58,0}}, color={0,0,127}));
  connect(adc_e.y, sub1.u2) annotation (Line(points={{-79,-50},{-72,-50},{-72,-46},
          {-62,-46}}, color={0,0,127}));
  connect(e_e.y, sub1.u1) annotation (Line(points={{-79,-28},{-72,-28},{-72,-34},
          {-62,-34}}, color={0,0,127}));
  connect(sub1.y, feedback.u2) annotation (Line(points={{-39,-40},{-36,-40},{-36,
          -20},{-50,-20},{-50,-8}}, color={0,0,127}));
  connect(pDT1.y, add1.u1)
    annotation (Line(points={{1,0},{10,0},{10,-4},{18,-4}}, color={0,0,127}));
  connect(v_e.y, add1.u2) annotation (Line(points={{1,-20},{10,-20},{10,-16},{18,
          -16}}, color={0,0,127}));
  connect(add1.y, gain.u)
    annotation (Line(points={{41,-10},{58,-10}}, color={0,0,127}));
  connect(realToInteger.y, pwm.u[1]) annotation (Line(points={{61,-50},{70,-50},
          {70,-50},{78,-50}}, color={255,127,0}));
  connect(gain.y, realToInteger.u) annotation (Line(points={{81,-10},{90,-10},{
          90,-30},{28,-30},{28,-50},{38,-50}}, color={0,0,127}));
  annotation (                                                                             Experiment(Interval = 0.01), Documentation(info = "<html>
<h4>Blink</h4>
<p>Blink is a very simple Arduino model, which simply toggles the built-in LED on the board on and off at the given frequency (this version blinks at a frequency given by the model using single precision floating point; it is also possible to simply flip the LED bit in each time step which gives a more accurate result). Use this model to see if your Modelica tool can export code for AVR MCUs.</p>
<p>Arduino digital pin 13 corresponds to digital pin B5 on the ATmega328P. If desired, you can connect an external LED to this PIN, with a suitable resistor in-between (perhaps 220&#8486;). Connect the other PIN on the LED to ground.</p>
<p>See also the <a href=\"https://www.arduino.cc/en/tutorial/blink\">Arduino tutorial</a> corresponding to this model.</p>
<p>Please see the <code><a href=\"modelica://Modelica_DeviceDrivers.EmbeddedTargets.AVR\">AVR</a></code> package documentation before testing the example!</p>
</html>"),
    Diagram(graphics={        Text(
          extent={{-94,-68},{96,-96}},
          lineColor={0,0,255},
          textString=
              "Please see the AVR package documentation before testing the example!")}));
end MagLev;
