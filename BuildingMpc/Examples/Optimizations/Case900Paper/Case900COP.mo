within BuildingMpc.Examples.Optimizations.Case900Paper;
model Case900COP
  extends BuildingMpc.Examples.SimulationModels.Case900Paper.Case900Paper(optVar2(y=
         mpc.u2), optVar1(y=mpc.u1),
    TRet(T_start=283.15),
    borFie(r=cat(
          1,
          lay.rC,
          {lay.r_b}), nbTem=11));

  Real buiStates[37];
  Real capStates[20];
  Real watStates[20];
  MpcCase900COP mpc(stateEstimationType=UnitTests.MPC.BaseClasses.StateEstimationType.Perfect);


  model MpcCase900COP
    extends UnitTests.MPC.BaseClasses.Mpc(
      final nOut=16,
      final nOpt=5,
      final nSta=188,
      final nMeas=0,
      final nIneq=7,
      final nLLIn=0,
      final nLLOut=0,
      final nLLSta=0,
      final horizonLength=360,
      final numControlIntervals=14,
      final controlTimeStep=1200,
      final nModCorCoeff=33,
      final name= "Case900COP");
    Modelica.Blocks.Interfaces.RealOutput COP = getOutput(tableID,6, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Q_con_max = getOutput(tableID,7, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Q_cond = getOutput(tableID,3, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput TRad = getOutput(tableID,4, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput TRet = getOutput(tableID,5, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput T_con_in = getOutput(tableID,8, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput T_con_out = getOutput(tableID,9, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput T_eva_in = getOutput(tableID,10, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput T_eva_out = getOutput(tableID,11, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,1, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput W_comp = getOutput(tableID,2, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput slack[3] = {
      getOutput(tableID,14, time),
      getOutput(tableID,15, time),
      getOutput(tableID,16, time)}
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,13, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,12, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
  end MpcCase900COP;

  IBPSA.Fluid.Geothermal.Borefields.BaseClasses.HeatTransfer.Cylindrical lay(
    each soiDat=borFieDat.soiDat,
    each r_a=borFieDat.conDat.rBor,
    each steadyStateInitial=false,
    each h=borFieDat.conDat.hBor/borFie.nSeg,
    each r_b=2*sqrt(borFieDat.soiDat.aSoi*604800),
    TInt_start=fixedTemperature.T,
    TExt_start=fixedTemperature.T)
                  annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={124,76})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature
    annotation (Placement(transformation(extent={{160,66},{140,86}})));
equation
   buiStates = {
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
rectangularZoneTemplate.radDistr.TRad}
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));

capStates = {
borFie.borHol.intHex[1].intResUTub.capFil1.T,
borFie.borHol.intHex[1].intResUTub.capFil2.T,
borFie.borHol.intHex[2].intResUTub.capFil1.T,
borFie.borHol.intHex[2].intResUTub.capFil2.T,
borFie.borHol.intHex[3].intResUTub.capFil1.T,
borFie.borHol.intHex[3].intResUTub.capFil2.T,
borFie.borHol.intHex[4].intResUTub.capFil1.T,
borFie.borHol.intHex[4].intResUTub.capFil2.T,
borFie.borHol.intHex[5].intResUTub.capFil1.T,
borFie.borHol.intHex[5].intResUTub.capFil2.T,
borFie.borHol.intHex[6].intResUTub.capFil1.T,
borFie.borHol.intHex[6].intResUTub.capFil2.T,
borFie.borHol.intHex[7].intResUTub.capFil1.T,
borFie.borHol.intHex[7].intResUTub.capFil2.T,
borFie.borHol.intHex[8].intResUTub.capFil1.T,
borFie.borHol.intHex[8].intResUTub.capFil2.T,
borFie.borHol.intHex[9].intResUTub.capFil1.T,
borFie.borHol.intHex[9].intResUTub.capFil2.T,
borFie.borHol.intHex[10].intResUTub.capFil1.T,
borFie.borHol.intHex[10].intResUTub.capFil2.T};

watStates = {
borFie.borHol.intHex[1].vol1.T,
borFie.borHol.intHex[1].vol2.T,
borFie.borHol.intHex[2].vol1.T,
borFie.borHol.intHex[2].vol2.T,
borFie.borHol.intHex[3].vol1.T,
borFie.borHol.intHex[3].vol2.T,
borFie.borHol.intHex[4].vol1.T,
borFie.borHol.intHex[4].vol2.T,
borFie.borHol.intHex[5].vol1.T,
borFie.borHol.intHex[5].vol2.T,
borFie.borHol.intHex[6].vol1.T,
borFie.borHol.intHex[6].vol2.T,
borFie.borHol.intHex[7].vol1.T,
borFie.borHol.intHex[7].vol2.T,
borFie.borHol.intHex[8].vol1.T,
borFie.borHol.intHex[8].vol2.T,
borFie.borHol.intHex[9].vol1.T,
borFie.borHol.intHex[9].vol2.T,
borFie.borHol.intHex[10].vol1.T,
borFie.borHol.intHex[10].vol2.T};

for i in 1:37 loop
        mpc.uSta[i] = buiStates[i];
end for;
for i in 1:20 loop
        mpc.uSta[i+37] = capStates[i];
        mpc.uSta[i+168] = watStates[i];
end for;mpc.uSta[168] = rectangularZoneTemplate.airModel.vol.T;
        for i in 1:10 loop
          for j in 1:10 loop
                  mpc.uSta[57+j+10*(i-1)] = borFie.TSoi[1+j,i];
          end for;
          mpc.uSta[157+i] = borFie.TSoi[12,i];
        end for;

  connect(lay.port_b, fixedTemperature.port)
    annotation (Line(points={{134,76},{140,76}}, color={191,0,0}));
  connect(lay.port_a, fixedTemperature.port) annotation (Line(points={{114,76},{
          106,76},{106,60},{140,60},{140,76}}, color={191,0,0}));
                          annotation (Placement(transformation(extent={{-100,0},{-80,20}})));
end Case900COP;
