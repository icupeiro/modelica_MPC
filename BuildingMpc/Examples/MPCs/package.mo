within BuildingMpc.Examples;
package MPCs
extends Modelica.Icons.ExamplesPackage;






  model MpcCase900TABS_HP
    extends UnitTests.MPC.BaseClasses.Mpc(
      final nOut=6,
      final nOpt=4,
      final nSta=30,
      final nMeas=0,
      final controlTimeStep=3600,
      final nModCorCoeff=21,
      final name= "Case900TABS_HP");
    Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,1, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,2, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u3 = getOutput(tableID,3, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,4, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,5, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput W_comp = getOutput(tableID,6, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
  end MpcCase900TABS_HP;
end MPCs;
