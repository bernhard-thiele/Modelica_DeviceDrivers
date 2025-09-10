within Modelica_DeviceDrivers.Blocks.Examples;
model TestSerialPackager_TCPIPServerClient
  "Example for combining TCP/IP server and client blocks."
  extends Modelica.Icons.Example;
  inner Modelica_DeviceDrivers.Blocks.Communication.TCPIPServerConfig
    tcpipserverconfig(
    port=10002,
    maxClients=1,
    useNonblockingMode=true)
    annotation (Placement(transformation(extent={{-80,-100},{-60,-80}})));
  Modelica_DeviceDrivers.Blocks.Communication.TCPIPServerReceive tCPIPReceive(
    clientIndex=1,
    blockUntilConnected=false,
                   showAdvancedOutputs=true)
                                annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-80,30})));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.GetInteger getInteger(n=3)
    annotation (Placement(transformation(extent={{-90,-20},{-70,0}})));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.Packager packager(
      useBackwardPropagatedBufferSize=false, userBufferSize=12)
    annotation (Placement(transformation(extent={{-30,20},{-10,40}})));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddInteger addInteger(n=3, nu=1)
    annotation (Placement(transformation(extent={{-30,-20},{-10,0}})));
  Modelica_DeviceDrivers.Blocks.Communication.TCPIPServerSend tCPIPSend(
    enableExternalTrigger=true,
    blockUntilConnected=false,
    autoBufferSize=true,
    userBufferSize=12) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-20,-50})));
  OperatingSystem.RealtimeSynchronize realtimeSynchronize
    annotation (Placement(transformation(extent={{-40,-100},{-20,-80}})));
  Process process
    annotation (Placement(transformation(extent={{-60,-20},{-40,0}})));
  Packaging.SerialPackager.Packager                               packager1(
      useBackwardPropagatedBufferSize=false, userBufferSize=12)                                                                      annotation(Placement(transformation(extent={{50,20},
            {70,40}})));
  Packaging.SerialPackager.AddInteger                               addInteger1(n=3, nu=1)
                                                                                     annotation(Placement(transformation(extent={{50,-20},
            {70,0}})));
  Modelica.Blocks.Sources.IntegerExpression intExp[3](y=integer(10*sin(time))*{
        1,2,3}) annotation (Placement(transformation(extent={{18,-20},{38,0}})));
  Packaging.SerialPackager.GetInteger                               getInteger1(n=3)
                                                                               annotation(Placement(transformation(extent={{50,-80},
            {70,-60}})));
  Communication.TCPIP_Client_IO tCPIP_Client_IO(
    startTime=1,
    port=10002,
    outputBufferSize=3*4,
    inputBufferSize=3*4,
    useNonblockingMode=true,
    serverIsReady=tcpipserverconfig.isReady)   annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={60,-40})));
protected
  block Process
    extends Modelica.Blocks.Icons.Block;
    Modelica.Blocks.Interfaces.BooleanInput trigger annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
    Modelica.Blocks.Interfaces.IntegerInput u[3]
      annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
    Modelica.Blocks.Interfaces.IntegerOutput y[3]
      annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  equation
    when trigger then
      Modelica.Utilities.Streams.print("Process, t="+String(time));
      y = 2*u;
    end when;
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Process;
equation
  connect(tCPIPReceive.pkgOut, getInteger.pkgIn)
    annotation (Line(points={{-80,19.2},{-80,0.8}}, color={0,0,0}));
  connect(packager.pkgOut, addInteger.pkgIn)
    annotation (Line(points={{-20,19.2},{-20,0.8}},color={0,0,0}));
  connect(addInteger.pkgOut[1], tCPIPSend.pkgIn)
    annotation (Line(points={{-20,-20.8},{-20,-39.2}},
                                                     color={0,0,0}));
  connect(tCPIPSend.trigger, tCPIPReceive.recvTrigger) annotation (Line(points={{-32,-50},
          {-92,-50},{-92,10},{-86,10},{-86,19}},           color={255,0,255}));
  connect(getInteger.y, process.u)
    annotation (Line(points={{-69,-10},{-62,-10}},
                                               color={255,127,0}));
  connect(process.y, addInteger.u)
    annotation (Line(points={{-39,-10},{-32,-10}},
                                             color={255,127,0}));
  connect(process.trigger, tCPIPReceive.recvTrigger) annotation (Line(points={{-50,-22},
          {-50,-50},{-92,-50},{-92,10},{-86,10},{-86,19}},    color={255,0,255}));
  connect(packager1.pkgOut, addInteger1.pkgIn)
    annotation (Line(points={{60,19.2},{60,0.8}},  color={0,0,0}));
  connect(addInteger1.pkgOut[1], tCPIP_Client_IO.pkgIn)
    annotation (Line(points={{60,-20.8},{60,-29.2}},
                                                   color={0,0,0}));
  connect(tCPIP_Client_IO.pkgOut, getInteger1.pkgIn)
    annotation (Line(points={{60,-50.8},{60,-59.2}}, color={0,0,0}));
  connect(intExp.y, addInteger1.u)
    annotation (Line(points={{39,-10},{48,-10}},
                                               color={255,127,0}));
  annotation (
    Documentation(info="<html>
<p>
The <code>tcpipserverconfig</code> block is configured for listening at port 10002 and for using a non-blocking TCP/IP
socket.
</p>
<p>
For meaningful results a TCP/IP client needs to connect and send suitable data, otherwise no data is received. Such
a client is provided as C code test program 
(<a href=\"modelica://Modelica_DeviceDrivers/Resources/test/Communication/TCPIPClientAsRemoteStation.c\">Resources/test/Communication/TCPIPClientAsRemoteStation.c</a>).
</p>
</html>"),
    experiment(
      StopTime=5,
      Interval=0.01,
      __Dymola_Algorithm="Euler"),
    Diagram(graphics={
        Text(
          extent={{-100,70},{0,50}},
          lineColor={28,108,200},
          textString="TCP/IP server
in non-blocking mode"),                                                       Line(
          points={{0,80},{0,-100},{0,-100}},
          color={238,46,47},
          pattern=LinePattern.Dash),
        Text(
          extent={{0,70},{100,50}},
          lineColor={28,108,200},
          textString="TCP/IP client
in non-blocking mode"),
        Text(
          extent={{0,-80},{100,-100}},
          lineColor={28,108,200},
          textString="Note: `tcpipserverconfig.isReady` output is bound
to `tCPIP_Client_IO.serverIsReady` input
for ensuring correct starting sequence"),
          Bitmap(extent={{-70,80},{-50,100}}, fileName="modelica://Modelica_DeviceDrivers/Resources/Images/Icons/Architetto----Esperiment-chimico.png"),
        Text(
          extent={{-50,100},{50,80}},
          lineColor={238,46,47},
          horizontalAlignment=TextAlignment.Left,
          textString="This is a bit experimental.
TODO Windows API updates!
TODO Debug mode with trace-prints for read/write etc
TODO Windows CAPI (TCPIPSocketClient_.connect_!)")}));
end TestSerialPackager_TCPIPServerClient;
