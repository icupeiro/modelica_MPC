within BuildingMpc.Examples.Optimizations.Case900Paper;
model Case900Sou
  extends BuildingMpc.Examples.SimulationModels.Case900Paper.Case900Paper(optVar2(y=
         mpc.u2), optVar1(y=mpc.u1),
    optVar3(y=mpc.u3));

  Real states[38];
  MpcCase900Sou mpc(stateEstimationType=UnitTests.MPC.BaseClasses.StateEstimationType.Perfect);

  model MpcCase900Sou
    extends UnitTests.MPC.BaseClasses.Mpc(
      final nOut=17,
      final nOpt=7,
      final nSta=38,
      final nMeas=0,
      final nIneq=7,
      final nLLIn=0,
      final nLLOut=0,
      final nLLSta=0,
      final horizonLength=168,
      final numControlIntervals=13,
      final controlTimeStep=3600,
      final nModCorCoeff=33,
      final name= "Case900Sou");
    Modelica.Blocks.Interfaces.RealOutput COP = getOutput(tableID,5, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Q_con_max = getOutput(tableID,6, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Q_cond = getOutput(tableID,3, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput TAir = getOutput(tableID,1, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput TRad = getOutput(tableID,4, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput T_con_in = getOutput(tableID,7, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput T_con_out = getOutput(tableID,8, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput T_eva_in = getOutput(tableID,9, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput T_eva_out = getOutput(tableID,10, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput W_comp = getOutput(tableID,2, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput slack[4] = {
      getOutput(tableID,13, time),
      getOutput(tableID,14, time),
      getOutput(tableID,15, time),
      getOutput(tableID,16, time)}
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,12, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,11, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u3 = getOutput(tableID,17, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
  end MpcCase900Sou;

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
rectangularZoneTemplate.airModel.vol.T}
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));

        mpc.uSta = states;

                          annotation (Placement(transformation(extent={{-100,0},{-80,20}})),
      experiment(
      StopTime=5259487,
      Interval=300,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=30,
      __Dymola_Algorithm="Euler"));
end Case900Sou;
