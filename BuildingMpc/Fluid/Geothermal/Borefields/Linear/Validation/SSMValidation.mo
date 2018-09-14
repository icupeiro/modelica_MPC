within BuildingMpc.Fluid.Geothermal.Borefields.Linear.Validation;
model SSMValidation
  extends Modelica.Icons.Example
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
  replaceable package Medium =
      IDEAS.Media.Water "Medium";
  parameter String fileName = Modelica.Utilities.Files.loadResource("modelica://BuildingMpc/Fluid/Geothermal/Borefields/Linear/SSM/ssm.mat");
  final parameter Integer nSta = Bsize[1] "Number of states";
  final parameter Integer nInp = Bsize[2] "Number of inputs";
  final parameter Integer nPreInp = 0 "Number of precomputed inputs";
  final parameter Integer nOut = Csize[1] "Number of precomputed outputs";


   Modelica.Blocks.Continuous.StateSpace stateSpace(
    A=readMatrix(
        fileName=fileName,
        matrixName="A",
        rows=nSta,
        columns=nSta),
    B=readMatrix(
        fileName=fileName,
        matrixName="B",
        rows=nSta,
        columns=nInp),
    C=readMatrix(
        fileName=fileName,
        matrixName="C",
        rows=nOut,
        columns=nSta),
    D=readMatrix(
        fileName=fileName,
        matrixName="D",
        rows=nOut,
        columns=nInp),
    x_start=x_start,
    y_start={273.15 + 13.5},
    initType=Modelica.Blocks.Types.Init.InitialState) "State space model"
     annotation (Placement(transformation(extent={{-20,20},{0,40}})));

protected
  parameter Real x_start[nSta](each fixed=false) "Initial state values";
  final parameter Integer[2] Bsize = readMatrixSize(fileName=fileName, matrixName="B") "Size of B matrix of state space model";
  final parameter Integer[2] Csize = readMatrixSize(fileName=fileName, matrixName="C") "Size of C matrix of state space model";
public
  Modelica.Blocks.Sources.Constant const(k=273.15 + 4)
    annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
  Modelica.Blocks.Sources.Constant const1(k=borFieDat.conDat.mBor_flow_nominal)
    annotation (Placement(transformation(extent={{-80,0},{-60,20}})));

  Development.OneUTube oneUTube(
    borFieDat=borFieDat,
    dp_nominal=0,
    redeclare package Medium = Medium,
    Tsoil=273.15 + 13.5,
    TGro_start=(273.15 + 13.5)*ones(10),
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal)
    annotation (Placement(transformation(extent={{-28,-64},{12,-24}})));
  IBPSA.Fluid.Sources.Boundary_pT sou(
    redeclare package Medium = Medium,
    use_p_in=false,
    nPorts=1,
    use_T_in=true,
    T=277.15) "Source" annotation (Placement(transformation(extent={{-86,-54},{-66,
            -34}},rotation=0)));
  IBPSA.Fluid.Sources.Boundary_pT sin(
    redeclare package Medium = Medium,
    use_p_in=false,
    use_T_in=false,
    nPorts=1) "Sink" annotation (Placement(transformation(extent={{96,-56},{76,
            -36}},rotation=0)));
  Buildings.Fluid.Movers.FlowControlled_m_flow mFlow(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    addPowerToMedium=false,
    m_flow_nominal=1,
    use_inputFilter=false)
    annotation (Placement(transformation(extent={{50,-54},{70,-34}})));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorIn1(
    redeclare package Medium = Medium,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    tau=0)                               "Inlet borehole temperature"
    annotation (Placement(transformation(extent={{-60,-54},{-40,-34}})));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorConOut(
    redeclare package Medium = Medium,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    tau=0)                               "Outlet borehole temperature"
    annotation (Placement(transformation(extent={{22,-54},{42,-34}})));
  IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example
    borFieDat(soiDat=IBPSA.Fluid.Geothermal.Borefields.Data.Soil.SandStone(
        steadyState=false), filDat=
        IBPSA.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(steadyState=false))
    annotation (Placement(transformation(extent={{40,60},{60,80}})));
  Modelica.Blocks.Math.Add add(k1=-1)
    annotation (Placement(transformation(extent={{14,28},{34,48}})));
public
  Modelica.Blocks.Sources.Constant const2(k=273.15)
    annotation (Placement(transformation(extent={{-20,60},{0,80}})));
initial algorithm
  x_start := (273.15+13.5)*ones(nSta);
equation
  connect(const.y, stateSpace.u[1]) annotation (Line(points={{-59,50},{-40,50},{
          -40,30},{-22,30}}, color={0,0,127}));
  connect(const1.y, stateSpace.u[2]) annotation (Line(points={{-59,10},{-40,10},
          {-40,30},{-22,30}}, color={0,0,127}));
  connect(oneUTube.port_b,TBorConOut. port_a)
    annotation (Line(points={{12,-44},{22,-44}},
                                            color={0,127,255}));
  connect(sou.ports[1],TBorIn1. port_a)
    annotation (Line(points={{-66,-44},{-60,-44}},
                                               color={0,127,255}));
  connect(oneUTube.port_a,TBorIn1. port_b)
    annotation (Line(points={{-28,-44},{-40,-44}},
                                               color={0,127,255}));
  connect(TBorConOut.port_b,mFlow. port_a)
    annotation (Line(points={{42,-44},{50,-44}},
                                             color={0,127,255}));
  connect(mFlow.port_b,sin. ports[1])
    annotation (Line(points={{70,-44},{74,-44},{74,-46},{76,-46}},
                                             color={0,127,255}));
  connect(const1.y, mFlow.m_flow_in)
    annotation (Line(points={{-59,10},{60,10},{60,-32}}, color={0,0,127}));
  connect(stateSpace.y[1], add.u2)
    annotation (Line(points={{1,30},{8,30},{8,32},{12,32}}, color={0,0,127}));
  connect(const.y, sou.T_in) annotation (Line(points={{-59,50},{-50,50},{-50,74},
          {-94,74},{-94,-40},{-88,-40}}, color={0,0,127}));
  connect(const2.y, add.u1)
    annotation (Line(points={{1,70},{8,70},{8,44},{12,44}}, color={0,0,127}));
end SSMValidation;
