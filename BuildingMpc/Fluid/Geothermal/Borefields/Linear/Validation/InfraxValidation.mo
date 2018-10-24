within BuildingMpc.Fluid.Geothermal.Borefields.Linear.Validation;
model InfraxValidation
  extends Modelica.Icons.Example
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
  package Medium = IBPSA.Media.Antifreeze.PropyleneGlycolWater(property_T=283.15, X_a=0.30);
  parameter String fileName = Modelica.Utilities.Files.loadResource("modelica://BuildingMpc/Fluid/Geothermal/Borefields/Linear/SSM/Infrax/ssm.mat");
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
    initType=Modelica.Blocks.Types.Init.NoInit) "State space model"
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
  INFRAX.Data.Parameters.BorefieldData.INFRAX_bF iNFRAX_bF
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
  TwoUTube twoUTube(
    borFieDat=iNFRAX_bF,
    allowFlowReversal=false,
    redeclare package Medium = Medium,
    TGro_start=(273.15 + 13.5)*ones(10),
    show_T=true,
    Tsoil=286.65)
    annotation (Placement(transformation(extent={{-30,-40},{10,0}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn(
    redeclare package Medium = Medium,
    tau=0,
    allowFlowReversal=false,
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal)
    annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOut(
    allowFlowReversal=false,
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal,
    redeclare package Medium = Medium,
    tau=0)
    annotation (Placement(transformation(extent={{20,-30},{40,-10}})));
  IDEAS.Fluid.Sources.Boundary_pT sou(
    redeclare package Medium = Medium,
    use_T_in=true,
    nPorts=1)
    annotation (Placement(transformation(extent={{-90,-30},{-70,-10}})));
  IDEAS.Fluid.Sources.Boundary_pT sink(redeclare package Medium = Medium,
      nPorts=1)
    annotation (Placement(transformation(extent={{100,-30},{80,-10}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource mFlow(
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04,
    redeclare final package Medium = Medium,
    control_dp=false) "Pressure source"
    annotation (Placement(transformation(extent={{50,-30},{70,-10}})));
public
  Modelica.Blocks.Sources.RealExpression realExpression2(y=twoUTube.borHol.sta_b.T)
    annotation (Placement(transformation(extent={{4,14},{24,34}})));
  IDEAS.Utilities.Time.ModelTime modTim
    annotation (Placement(transformation(extent={{-150,58},{-130,78}})));
  Modelica.Blocks.Tables.CombiTable1Ds combiTable2D(
    tableOnFile=true,
    tableName="data",
    fileName=ModelicaServices.ExternalReferences.loadResource("modelica://INFRAX/Resources/Validation/borefield_validation.txt"),
    columns={2,3,4,5,6,7})
    annotation (Placement(transformation(extent={{-108,58},{-88,78}})));

  Modelica.Blocks.Sources.Constant cToK(k=273.15)
    annotation (Placement(transformation(extent={{-76,46},{-64,58}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{-56,60},{-36,80}})));
public
  Modelica.Blocks.Sources.RealExpression realExpression1(y=Medium.d_const)
    annotation (Placement(transformation(extent={{56,68},{76,88}})));
  Modelica.Blocks.Math.Gain        cToK1(k=10/3600*Medium.d_const/1000)
    annotation (Placement(transformation(extent={{-18,18},{-6,30}})));
initial algorithm
   x_start := {
twoUTube.borHol.intHex[1].vol1.T,
twoUTube.borHol.intHex[1].vol2.T,
twoUTube.borHol.intHex[1].vol3.T,
twoUTube.borHol.intHex[1].vol4.T,
twoUTube.borHol.intHex[2].vol1.T,
twoUTube.borHol.intHex[2].vol2.T,
twoUTube.borHol.intHex[2].vol3.T,
twoUTube.borHol.intHex[2].vol4.T,
twoUTube.borHol.intHex[3].vol1.T,
twoUTube.borHol.intHex[3].vol2.T,
twoUTube.borHol.intHex[3].vol3.T,
twoUTube.borHol.intHex[3].vol4.T,
twoUTube.borHol.intHex[4].vol1.T,
twoUTube.borHol.intHex[4].vol2.T,
twoUTube.borHol.intHex[4].vol3.T,
twoUTube.borHol.intHex[4].vol4.T,
twoUTube.borHol.intHex[5].vol1.T,
twoUTube.borHol.intHex[5].vol2.T,
twoUTube.borHol.intHex[5].vol3.T,
twoUTube.borHol.intHex[5].vol4.T,
twoUTube.borHol.intHex[6].vol1.T,
twoUTube.borHol.intHex[6].vol2.T,
twoUTube.borHol.intHex[6].vol3.T,
twoUTube.borHol.intHex[6].vol4.T,
twoUTube.borHol.intHex[7].vol1.T,
twoUTube.borHol.intHex[7].vol2.T,
twoUTube.borHol.intHex[7].vol3.T,
twoUTube.borHol.intHex[7].vol4.T,
twoUTube.borHol.intHex[8].vol1.T,
twoUTube.borHol.intHex[8].vol2.T,
twoUTube.borHol.intHex[8].vol3.T,
twoUTube.borHol.intHex[8].vol4.T,
twoUTube.borHol.intHex[9].vol1.T,
twoUTube.borHol.intHex[9].vol2.T,
twoUTube.borHol.intHex[9].vol3.T,
twoUTube.borHol.intHex[9].vol4.T,
twoUTube.borHol.intHex[10].vol1.T,
twoUTube.borHol.intHex[10].vol2.T,
twoUTube.borHol.intHex[10].vol3.T,
twoUTube.borHol.intHex[10].vol4.T,
twoUTube.lay[1].T[1],
twoUTube.lay[1].T[2],
twoUTube.lay[1].T[3],
twoUTube.lay[1].T[4],
twoUTube.lay[1].T[5],
twoUTube.lay[1].T[6],
twoUTube.lay[1].T[7],
twoUTube.lay[1].T[8],
twoUTube.lay[1].T[9],
twoUTube.lay[1].T[10],
twoUTube.lay[2].T[1],
twoUTube.lay[2].T[2],
twoUTube.lay[2].T[3],
twoUTube.lay[2].T[4],
twoUTube.lay[2].T[5],
twoUTube.lay[2].T[6],
twoUTube.lay[2].T[7],
twoUTube.lay[2].T[8],
twoUTube.lay[2].T[9],
twoUTube.lay[2].T[10],
twoUTube.lay[3].T[1],
twoUTube.lay[3].T[2],
twoUTube.lay[3].T[3],
twoUTube.lay[3].T[4],
twoUTube.lay[3].T[5],
twoUTube.lay[3].T[6],
twoUTube.lay[3].T[7],
twoUTube.lay[3].T[8],
twoUTube.lay[3].T[9],
twoUTube.lay[3].T[10],
twoUTube.lay[4].T[1],
twoUTube.lay[4].T[2],
twoUTube.lay[4].T[3],
twoUTube.lay[4].T[4],
twoUTube.lay[4].T[5],
twoUTube.lay[4].T[6],
twoUTube.lay[4].T[7],
twoUTube.lay[4].T[8],
twoUTube.lay[4].T[9],
twoUTube.lay[4].T[10],
twoUTube.lay[5].T[1],
twoUTube.lay[5].T[2],
twoUTube.lay[5].T[3],
twoUTube.lay[5].T[4],
twoUTube.lay[5].T[5],
twoUTube.lay[5].T[6],
twoUTube.lay[5].T[7],
twoUTube.lay[5].T[8],
twoUTube.lay[5].T[9],
twoUTube.lay[5].T[10],
twoUTube.lay[6].T[1],
twoUTube.lay[6].T[2],
twoUTube.lay[6].T[3],
twoUTube.lay[6].T[4],
twoUTube.lay[6].T[5],
twoUTube.lay[6].T[6],
twoUTube.lay[6].T[7],
twoUTube.lay[6].T[8],
twoUTube.lay[6].T[9],
twoUTube.lay[6].T[10],
twoUTube.lay[7].T[1],
twoUTube.lay[7].T[2],
twoUTube.lay[7].T[3],
twoUTube.lay[7].T[4],
twoUTube.lay[7].T[5],
twoUTube.lay[7].T[6],
twoUTube.lay[7].T[7],
twoUTube.lay[7].T[8],
twoUTube.lay[7].T[9],
twoUTube.lay[7].T[10],
twoUTube.lay[8].T[1],
twoUTube.lay[8].T[2],
twoUTube.lay[8].T[3],
twoUTube.lay[8].T[4],
twoUTube.lay[8].T[5],
twoUTube.lay[8].T[6],
twoUTube.lay[8].T[7],
twoUTube.lay[8].T[8],
twoUTube.lay[8].T[9],
twoUTube.lay[8].T[10],
twoUTube.lay[9].T[1],
twoUTube.lay[9].T[2],
twoUTube.lay[9].T[3],
twoUTube.lay[9].T[4],
twoUTube.lay[9].T[5],
twoUTube.lay[9].T[6],
twoUTube.lay[9].T[7],
twoUTube.lay[9].T[8],
twoUTube.lay[9].T[9],
twoUTube.lay[9].T[10],
twoUTube.lay[10].T[1],
twoUTube.lay[10].T[2],
twoUTube.lay[10].T[3],
twoUTube.lay[10].T[4],
twoUTube.lay[10].T[5],
twoUTube.lay[10].T[6],
twoUTube.lay[10].T[7],
twoUTube.lay[10].T[8],
twoUTube.lay[10].T[9],
twoUTube.lay[10].T[10],
twoUTube.Cground[1].T,
twoUTube.Cground[2].T,
twoUTube.Cground[3].T,
twoUTube.Cground[4].T,
twoUTube.Cground[5].T,
twoUTube.Cground[6].T,
twoUTube.Cground[7].T,
twoUTube.Cground[8].T,
twoUTube.Cground[9].T,
twoUTube.Cground[10].T,
twoUTube.borHol.intHex[1].intRes2UTub.capFil1.T,
twoUTube.borHol.intHex[1].intRes2UTub.capFil2.T,
twoUTube.borHol.intHex[1].intRes2UTub.capFil3.T,
twoUTube.borHol.intHex[1].intRes2UTub.capFil4.T,
twoUTube.borHol.intHex[2].intRes2UTub.capFil1.T,
twoUTube.borHol.intHex[2].intRes2UTub.capFil2.T,
twoUTube.borHol.intHex[2].intRes2UTub.capFil3.T,
twoUTube.borHol.intHex[2].intRes2UTub.capFil4.T,
twoUTube.borHol.intHex[3].intRes2UTub.capFil1.T,
twoUTube.borHol.intHex[3].intRes2UTub.capFil2.T,
twoUTube.borHol.intHex[3].intRes2UTub.capFil3.T,
twoUTube.borHol.intHex[3].intRes2UTub.capFil4.T,
twoUTube.borHol.intHex[4].intRes2UTub.capFil1.T,
twoUTube.borHol.intHex[4].intRes2UTub.capFil2.T,
twoUTube.borHol.intHex[4].intRes2UTub.capFil3.T,
twoUTube.borHol.intHex[4].intRes2UTub.capFil4.T,
twoUTube.borHol.intHex[5].intRes2UTub.capFil1.T,
twoUTube.borHol.intHex[5].intRes2UTub.capFil2.T,
twoUTube.borHol.intHex[5].intRes2UTub.capFil3.T,
twoUTube.borHol.intHex[5].intRes2UTub.capFil4.T,
twoUTube.borHol.intHex[6].intRes2UTub.capFil1.T,
twoUTube.borHol.intHex[6].intRes2UTub.capFil2.T,
twoUTube.borHol.intHex[6].intRes2UTub.capFil3.T,
twoUTube.borHol.intHex[6].intRes2UTub.capFil4.T,
twoUTube.borHol.intHex[7].intRes2UTub.capFil1.T,
twoUTube.borHol.intHex[7].intRes2UTub.capFil2.T,
twoUTube.borHol.intHex[7].intRes2UTub.capFil3.T,
twoUTube.borHol.intHex[7].intRes2UTub.capFil4.T,
twoUTube.borHol.intHex[8].intRes2UTub.capFil1.T,
twoUTube.borHol.intHex[8].intRes2UTub.capFil2.T,
twoUTube.borHol.intHex[8].intRes2UTub.capFil3.T,
twoUTube.borHol.intHex[8].intRes2UTub.capFil4.T,
twoUTube.borHol.intHex[9].intRes2UTub.capFil1.T,
twoUTube.borHol.intHex[9].intRes2UTub.capFil2.T,
twoUTube.borHol.intHex[9].intRes2UTub.capFil3.T,
twoUTube.borHol.intHex[9].intRes2UTub.capFil4.T,
twoUTube.borHol.intHex[10].intRes2UTub.capFil1.T,
twoUTube.borHol.intHex[10].intRes2UTub.capFil2.T,
twoUTube.borHol.intHex[10].intRes2UTub.capFil3.T,
twoUTube.borHol.intHex[10].intRes2UTub.capFil4.T};
equation
  connect(stateSpace.y[1], err.u1) annotation (Line(points={{1,70},{30,70},{30,36},
          {38,36}},     color={0,0,127}));
  connect(err.y, error)
    annotation (Line(points={{61,30},{110,30}}, color={0,0,127}));
  connect(TIn.port_b,twoUTube. port_a)
    annotation (Line(points={{-40,-20},{-30,-20}}, color={0,127,255}));
  connect(twoUTube.port_b, TOut.port_a)
    annotation (Line(points={{10,-20},{20,-20}}, color={0,127,255}));
  connect(TOut.port_b, mFlow.port_a)
    annotation (Line(points={{40,-20},{50,-20}}, color={0,127,255}));
  connect(mFlow.port_b, sink.ports[1])
    annotation (Line(points={{70,-20},{80,-20}}, color={0,127,255}));
  connect(sou.ports[1], TIn.port_a)
    annotation (Line(points={{-70,-20},{-60,-20}}, color={0,127,255}));
  connect(realExpression2.y, err.u2)
    annotation (Line(points={{25,24},{38,24}}, color={0,0,127}));
  connect(combiTable2D.y[2],add. u1) annotation (Line(points={{-87,68},{-82,68},
          {-82,76},{-58,76}},
                        color={0,0,127}));
  connect(modTim.y, combiTable2D.u)
    annotation (Line(points={{-129,68},{-110,68}}, color={0,0,127}));
  connect(cToK.y, add.u2) annotation (Line(points={{-63.4,52},{-62,52},{-62,64},
          {-58,64}}, color={0,0,127}));
  connect(add.y, stateSpace.u[1])
    annotation (Line(points={{-35,70},{-22,70}}, color={0,0,127}));
  connect(add.y, sou.T_in) annotation (Line(points={{-35,70},{-30,70},{-30,38},{
          -96,38},{-96,-16},{-92,-16}}, color={0,0,127}));
  connect(cToK1.y, mFlow.m_flow_in) annotation (Line(points={{-5.4,24},{0,24},{0,
          4},{54,4},{54,-12},{54,-12}}, color={0,0,127}));
  connect(combiTable2D.y[3], cToK1.u) annotation (Line(points={{-87,68},{-84,68},
          {-84,18},{-19.2,18},{-19.2,24}}, color={0,0,127}));
  annotation (
    experiment(
      StartTime=480,
      StopTime=17365920,
      Interval=480,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=5,
      __Dymola_Algorithm="Euler"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(
      Advanced(
        GenerateVariableDependencies=false,
        OutputModelicaCode=false,
        InlineMethod=0,
        InlineOrder=2,
        InlineFixedStep=0.001),
      Evaluate=false,
      OutputCPUtime=true,
      OutputFlatModelica=false));
end InfraxValidation;
