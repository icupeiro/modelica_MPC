within BuildingMpc.Examples.Optimizations.Case900Paper;
model Case900SouStaUpd
  extends BuildingMpc.Examples.SimulationModels.Case900Paper.Case900Paper(optVar2(y=
         mpc.u2), optVar1(y=mpc.u1),
    sim(lineariseJModelica=false),
    TRet(T_start=283.15));

  Real states[39];
  MpcCase900SouStaUpd mpc(stateEstimationType=UnitTests.MPC.BaseClasses.StateEstimationType.Perfect);

    model MpcCase900SouStaUpd
    extends UnitTests.MPC.BaseClasses.Mpc(
      final nOut=10,
      final nOpt=3,
      final nSta=39,
      final nMeas=0,
      final nIneq=7,
      final nLLIn=0,
      final nLLOut=0,
      final nLLSta=0,
      final horizonLength=360,
      final numControlIntervals=14,
      final controlTimeStep=1200,
      final nModCorCoeff=33,
      final name= "Case900SouStaUpd");
    Modelica.Blocks.Interfaces.RealOutput COP = getOutput(tableID,6, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Q_con_max = getOutput(tableID,7, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Q_cond = getOutput(tableID,3, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput TRad = getOutput(tableID,5, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput TRet = getOutput(tableID,4, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,1, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput W_comp = getOutput(tableID,2, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,10, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,9, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,8, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    end MpcCase900SouStaUpd;

equation
   states = {
rectangularZoneTemplate.winA.heaCapGla.T,
rectangularZoneTemplate.winB.heaCapGla.T,
rectangularZoneTemplate.winC.heaCapGla.T,
rectangularZoneTemplate.winD.heaCapGla.T,
rectangularZoneTemplate.bouFlo.layMul.port_gain[1].T,
rectangularZoneTemplate.bouFlo.layMul.port_b.T,
embeddedPipe.heatPortEmb[1].T,
rectangularZoneTemplate.propsBusInt[5].surfRad.T,
rectangularZoneTemplate.outA.port_emb[1].T,
rectangularZoneTemplate.outA.extCon.port_a.T,
rectangularZoneTemplate.outA.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[1].surfRad.T,
rectangularZoneTemplate.outA.layMul.monLay[3].monLayDyn.T[2],
rectangularZoneTemplate.outB.port_emb[1].T,
rectangularZoneTemplate.outB.extCon.port_a.T,
rectangularZoneTemplate.outB.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[2].surfRad.T,
rectangularZoneTemplate.outB.layMul.monLay[3].monLayDyn.T[2],
rectangularZoneTemplate.outC.port_emb[1].T,
rectangularZoneTemplate.outC.extCon.port_a.T,
rectangularZoneTemplate.outC.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[3].surfRad.T,
rectangularZoneTemplate.outC.layMul.monLay[3].monLayDyn.T[2],
rectangularZoneTemplate.outD.port_emb[1].T,
rectangularZoneTemplate.outD.extCon.port_a.T,
rectangularZoneTemplate.outD.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[4].surfRad.T,
rectangularZoneTemplate.outD.layMul.monLay[3].monLayDyn.T[2],
rectangularZoneTemplate.outCei.port_emb[1].T,
rectangularZoneTemplate.outCei.extCon.port_a.T,
rectangularZoneTemplate.outCei.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[6].surfRad.T,
rectangularZoneTemplate.int.port_emb[1].T,
rectangularZoneTemplate.propsBusInt[8].surfRad.T,
rectangularZoneTemplate.int.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[7].surfRad.T,
rectangularZoneTemplate.radDistr.TRad,
TRet.T,
rectangularZoneTemplate.airModel.vol.T}
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));

        mpc.uSta = states;

                          annotation (Placement(transformation(extent={{-100,0},{-80,20}})),
      experiment(
      StopTime=5259487,
      __Dymola_NumberOfIntervals=15000,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=30,
      __Dymola_Algorithm="Euler"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(
      Advanced(GenerateVariableDependencies=false, OutputModelicaCode=false),
      Evaluate=false,
      OutputCPUtime=false,
      OutputFlatModelica=false));
end Case900SouStaUpd;
