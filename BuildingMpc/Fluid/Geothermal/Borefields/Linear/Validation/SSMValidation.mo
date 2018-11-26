within BuildingMpc.Fluid.Geothermal.Borefields.Linear.Validation;
model SSMValidation
  extends Modelica.Icons.Example
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
  replaceable package Medium =
      IDEAS.Media.Water "Medium";
  parameter String fileName = Modelica.Utilities.Files.loadResource("modelica://BuildingMpc/Fluid/Geothermal/Borefields/Linear/SSM/Example/ssm.mat");
  parameter Modelica.SIunits.TemperatureDifference dT = 3;
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
    initType=Modelica.Blocks.Types.Init.InitialState)
                                                "State space model"
     annotation (Placement(transformation(extent={{-20,60},{0,80}})));

protected
  parameter Real x_start[nSta](each fixed=false) "Initial state values";
  final parameter Integer[2] Bsize = readMatrixSize(fileName=fileName, matrixName="B") "Size of B matrix of state space model";
  final parameter Integer[2] Csize = readMatrixSize(fileName=fileName, matrixName="C") "Size of C matrix of state space model";

public
  Modelica.Blocks.Math.Add err(k1=-1)
    annotation (Placement(transformation(extent={{40,20},{60,40}})));
  Modelica.Blocks.Interfaces.RealOutput error
    annotation (Placement(transformation(extent={{100,20},{120,40}})));
  IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example borFieDat
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
  OneUTube oneUTube(
    borFieDat=borFieDat,
    m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal,
    dp_nominal=0,
    allowFlowReversal=false,
    redeclare package Medium = Medium,
    TGro_start=(273.15 + 13.5)*ones(10),
    Tsoil=286.65,
    show_T=true)
    annotation (Placement(transformation(extent={{-30,-40},{10,0}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn(
    redeclare package Medium = Medium,
    tau=0,
    allowFlowReversal=false,
    m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal)
    annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOut(
    allowFlowReversal=false,
    m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal,
    redeclare package Medium = Medium,
    tau=0)
    annotation (Placement(transformation(extent={{20,-30},{40,-10}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource mFlow(
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04,
    redeclare final package Medium = Medium,
    control_dp=false) "Pressure source"
    annotation (Placement(transformation(extent={{56,-30},{76,-10}})));
public
  Modelica.Blocks.Sources.Constant const(k=1000)
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  IDEAS.Fluid.Sources.Boundary_pT bou(redeclare package Medium = Medium, nPorts=
       1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-90,10})));
protected
  IDEAS.Fluid.HeatExchangers.HeaterCooler_u
                                          outCon(
    final massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    redeclare final package Medium = Medium,
    final allowFlowReversal=false,
    final m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal,
    final tau=0,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    dp_nominal=borFieDat.conDat.dp_nominal,
    Q_flow_nominal=1)
                   "Model to set outlet conditions"
    annotation (Placement(transformation(extent={{12,-82},{-14,-56}})));
public
  Modelica.Blocks.Math.Gain gain(k=1/(Medium.cp_const*dT))
    annotation (Placement(transformation(extent={{-38,6},{-18,26}})));
initial algorithm
  x_start := (273.15+13.5)*ones(nSta);
//   x_start := {
// oneUTube.borHol.intHex[1].vol1.T,
// oneUTube.borHol.intHex[1].vol2.T,
// oneUTube.borHol.intHex[2].vol1.T,
// oneUTube.borHol.intHex[2].vol2.T,
// oneUTube.borHol.intHex[3].vol1.T,
// oneUTube.borHol.intHex[3].vol2.T,
// oneUTube.borHol.intHex[4].vol1.T,
// oneUTube.borHol.intHex[4].vol2.T,
// oneUTube.borHol.intHex[5].vol1.T,
// oneUTube.borHol.intHex[5].vol2.T,
// oneUTube.borHol.intHex[6].vol1.T,
// oneUTube.borHol.intHex[6].vol2.T,
// oneUTube.borHol.intHex[7].vol1.T,
// oneUTube.borHol.intHex[7].vol2.T,
// oneUTube.borHol.intHex[8].vol1.T,
// oneUTube.borHol.intHex[8].vol2.T,
// oneUTube.borHol.intHex[9].vol1.T,
// oneUTube.borHol.intHex[9].vol2.T,
// oneUTube.borHol.intHex[10].vol1.T,
// oneUTube.borHol.intHex[10].vol2.T,
// oneUTube.lay[1].Csoil[1].T,
// oneUTube.lay[1].Csoil[2].T,
// oneUTube.lay[1].Csoil[3].T,
// oneUTube.lay[1].Csoil[4].T,
// oneUTube.lay[1].Csoil[5].T,
// oneUTube.lay[1].Csoil[6].T,
// oneUTube.lay[1].Csoil[7].T,
// oneUTube.lay[1].Csoil[8].T,
// oneUTube.lay[1].Csoil[9].T,
// oneUTube.lay[1].Csoil[10].T,
// oneUTube.lay[2].Csoil[1].T,
// oneUTube.lay[2].Csoil[2].T,
// oneUTube.lay[2].Csoil[3].T,
// oneUTube.lay[2].Csoil[4].T,
// oneUTube.lay[2].Csoil[5].T,
// oneUTube.lay[2].Csoil[6].T,
// oneUTube.lay[2].Csoil[7].T,
// oneUTube.lay[2].Csoil[8].T,
// oneUTube.lay[2].Csoil[9].T,
// oneUTube.lay[2].Csoil[10].T,
// oneUTube.lay[3].Csoil[1].T,
// oneUTube.lay[3].Csoil[2].T,
// oneUTube.lay[3].Csoil[3].T,
// oneUTube.lay[3].Csoil[4].T,
// oneUTube.lay[3].Csoil[5].T,
// oneUTube.lay[3].Csoil[6].T,
// oneUTube.lay[3].Csoil[7].T,
// oneUTube.lay[3].Csoil[8].T,
// oneUTube.lay[3].Csoil[9].T,
// oneUTube.lay[3].Csoil[10].T,
// oneUTube.lay[4].Csoil[1].T,
// oneUTube.lay[4].Csoil[2].T,
// oneUTube.lay[4].Csoil[3].T,
// oneUTube.lay[4].Csoil[4].T,
// oneUTube.lay[4].Csoil[5].T,
// oneUTube.lay[4].Csoil[6].T,
// oneUTube.lay[4].Csoil[7].T,
// oneUTube.lay[4].Csoil[8].T,
// oneUTube.lay[4].Csoil[9].T,
// oneUTube.lay[4].Csoil[10].T,
// oneUTube.lay[5].Csoil[1].T,
// oneUTube.lay[5].Csoil[2].T,
// oneUTube.lay[5].Csoil[3].T,
// oneUTube.lay[5].Csoil[4].T,
// oneUTube.lay[5].Csoil[5].T,
// oneUTube.lay[5].Csoil[6].T,
// oneUTube.lay[5].Csoil[7].T,
// oneUTube.lay[5].Csoil[8].T,
// oneUTube.lay[5].Csoil[9].T,
// oneUTube.lay[5].Csoil[10].T,
// oneUTube.lay[6].Csoil[1].T,
// oneUTube.lay[6].Csoil[2].T,
// oneUTube.lay[6].Csoil[3].T,
// oneUTube.lay[6].Csoil[4].T,
// oneUTube.lay[6].Csoil[5].T,
// oneUTube.lay[6].Csoil[6].T,
// oneUTube.lay[6].Csoil[7].T,
// oneUTube.lay[6].Csoil[8].T,
// oneUTube.lay[6].Csoil[9].T,
// oneUTube.lay[6].Csoil[10].T,
// oneUTube.lay[7].Csoil[1].T,
// oneUTube.lay[7].Csoil[2].T,
// oneUTube.lay[7].Csoil[3].T,
// oneUTube.lay[7].Csoil[4].T,
// oneUTube.lay[7].Csoil[5].T,
// oneUTube.lay[7].Csoil[6].T,
// oneUTube.lay[7].Csoil[7].T,
// oneUTube.lay[7].Csoil[8].T,
// oneUTube.lay[7].Csoil[9].T,
// oneUTube.lay[7].Csoil[10].T,
// oneUTube.lay[8].Csoil[1].T,
// oneUTube.lay[8].Csoil[2].T,
// oneUTube.lay[8].Csoil[3].T,
// oneUTube.lay[8].Csoil[4].T,
// oneUTube.lay[8].Csoil[5].T,
// oneUTube.lay[8].Csoil[6].T,
// oneUTube.lay[8].Csoil[7].T,
// oneUTube.lay[8].Csoil[8].T,
// oneUTube.lay[8].Csoil[9].T,
// oneUTube.lay[8].Csoil[10].T,
// oneUTube.lay[9].Csoil[1].T,
// oneUTube.lay[9].Csoil[2].T,
// oneUTube.lay[9].Csoil[3].T,
// oneUTube.lay[9].Csoil[4].T,
// oneUTube.lay[9].Csoil[5].T,
// oneUTube.lay[9].Csoil[6].T,
// oneUTube.lay[9].Csoil[7].T,
// oneUTube.lay[9].Csoil[8].T,
// oneUTube.lay[9].Csoil[9].T,
// oneUTube.lay[9].Csoil[10].T,
// oneUTube.lay[10].Csoil[1].T,
// oneUTube.lay[10].Csoil[2].T,
// oneUTube.lay[10].Csoil[3].T,
// oneUTube.lay[10].Csoil[4].T,
// oneUTube.lay[10].Csoil[5].T,
// oneUTube.lay[10].Csoil[6].T,
// oneUTube.lay[10].Csoil[7].T,
// oneUTube.lay[10].Csoil[8].T,
// oneUTube.lay[10].Csoil[9].T,
// oneUTube.lay[10].Csoil[10].T,
// oneUTube.Cground[1].T,
// oneUTube.Cground[2].T,
// oneUTube.Cground[3].T,
// oneUTube.Cground[4].T,
// oneUTube.Cground[5].T,
// oneUTube.Cground[6].T,
// oneUTube.Cground[7].T,
// oneUTube.Cground[8].T,
// oneUTube.Cground[9].T,
// oneUTube.Cground[10].T,
// oneUTube.borHol.intHex[1].intResUTub.capFil1.T,
// oneUTube.borHol.intHex[1].intResUTub.capFil2.T,
// oneUTube.borHol.intHex[2].intResUTub.capFil1.T,
// oneUTube.borHol.intHex[2].intResUTub.capFil2.T,
// oneUTube.borHol.intHex[3].intResUTub.capFil1.T,
// oneUTube.borHol.intHex[3].intResUTub.capFil2.T,
// oneUTube.borHol.intHex[4].intResUTub.capFil1.T,
// oneUTube.borHol.intHex[4].intResUTub.capFil2.T,
// oneUTube.borHol.intHex[5].intResUTub.capFil1.T,
// oneUTube.borHol.intHex[5].intResUTub.capFil2.T,
// oneUTube.borHol.intHex[6].intResUTub.capFil1.T,
// oneUTube.borHol.intHex[6].intResUTub.capFil2.T,
// oneUTube.borHol.intHex[7].intResUTub.capFil1.T,
// oneUTube.borHol.intHex[7].intResUTub.capFil2.T,
// oneUTube.borHol.intHex[8].intResUTub.capFil1.T,
// oneUTube.borHol.intHex[8].intResUTub.capFil2.T,
// oneUTube.borHol.intHex[9].intResUTub.capFil1.T,
// oneUTube.borHol.intHex[9].intResUTub.capFil2.T,
// oneUTube.borHol.intHex[10].intResUTub.capFil1.T,
// oneUTube.borHol.intHex[10].intResUTub.capFil2.T};
equation
  connect(stateSpace.y[1], err.u1) annotation (Line(points={{1,70},{30,70},{30,36},
          {38,36}},     color={0,0,127}));
  connect(err.y, error)
    annotation (Line(points={{61,30},{110,30}}, color={0,0,127}));
  connect(TIn.port_b, oneUTube.port_a)
    annotation (Line(points={{-40,-20},{-30,-20}}, color={0,127,255}));
  connect(oneUTube.port_b, TOut.port_a)
    annotation (Line(points={{10,-20},{20,-20}}, color={0,127,255}));
  connect(TOut.port_b, mFlow.port_a)
    annotation (Line(points={{40,-20},{56,-20}}, color={0,127,255}));
  connect(const.y, stateSpace.u[1])
    annotation (Line(points={{-59,70},{-22,70}}, color={0,0,127}));
  connect(TOut.T, err.u2)
    annotation (Line(points={{30,-9},{30,24},{38,24}}, color={0,0,127}));
  connect(bou.ports[1], TIn.port_a) annotation (Line(points={{-90,0},{-90,-12},{
          -60,-12},{-60,-20}}, color={0,127,255}));
  connect(TIn.port_a, outCon.port_b) annotation (Line(points={{-60,-20},{-90,
          -20},{-90,-69},{-14,-69}}, color={0,127,255}));
  connect(mFlow.port_b, outCon.port_a) annotation (Line(points={{76,-20},{86,
          -20},{86,-69},{12,-69}}, color={0,127,255}));
  connect(outCon.Q_flow, gain.u) annotation (Line(points={{-15.3,-61.2},{-56,
          -61.2},{-56,16},{-40,16}}, color={0,0,127}));
  connect(gain.y, mFlow.m_flow_in)
    annotation (Line(points={{-17,16},{60,16},{60,-12}}, color={0,0,127}));
  connect(const.y, outCon.u) annotation (Line(points={{-59,70},{-40,70},{-40,40},
          {22,40},{22,-61.2},{14.6,-61.2}}, color={0,0,127}));
end SSMValidation;
