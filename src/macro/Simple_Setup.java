package macro;

import java.util.*;

import star.base.neo.StringVector;
import star.cadmodeler.*;
import star.common.*;
import star.flow.*;
import star.mapping.SolutionInterpolationModel;
import star.material.*;
import star.meshing.*;
import star.metrics.ThreeDimensionalModel;
import star.prismmesher.NumPrismLayers;
import star.prismmesher.PrismLayerStretching;
import star.prismmesher.PrismThickness;
import star.turbulence.*;
import star.coupledflow.CoupledFlowModel;
import star.keturb.*;

public class Simple_Setup extends StarMacro {

    // Global Map to store your "Pre-Known" values
    private Map<String, Double> configMap = new HashMap<>();

    @Override
    public void execute() {
        Simulation sim = getActiveSimulation();
        sim.println("--- Starting Automation ---");

        // 0. [USER INPUT] ENTER YOUR KNOWN VALUES HERE
        configureInputs(sim);

        // 1. PARAMETERS
        createGlobalParameters(sim);

        // 2. GEOMETRY
        GeometryPart myPart = createPartsGeneric(sim);
        if (myPart == null) return;

        // SCAN GEOMETRY & CREATE BOUNDARY PARAMETERS
        createDynamicParameters(sim, myPart);

        // 3. PHYSICS & MATERIALS
        setupPhysicsAndMaterials(sim);

        // INITIAL CONDITIONS
        setupInitialConditions(sim);

        // 4. REGION & BOUNDARIES
        setupRegionAndBoundaries(sim, myPart);

        // 5. MESH SETUP
        setupMesh(sim, myPart);

        // 6. CUSTOM CONTROLS
        // Retrieve the mesh op we just created to pass it to the control
        AutoMeshOperation meshOp = (AutoMeshOperation) sim.get(MeshOperationManager.class).getObject("Automated Mesh");
        addControlToAllSurfaces(sim, meshOp, myPart);

        // 7. GENERATE MESH
        //sim.println("--- Generating Volume Mesh ---");
        //sim.getMeshPipelineController().generateVolumeMesh();

        sim.println("--- Automation Complete ---");
    }

    // ==========================================================
    // USER CONFIGURATION
    // ==========================================================
    private void configureInputs(Simulation sim) {
        sim.println("--- Loading User Configuration ---");

        // Define the KNOWN values in advance
        // The key must match the pattern: Inlet_Velocity_ID or Outlet_Pressure_ID
        configMap.put("Inlet_Velocity_1", 20.0); // value for inlet 1
        configMap.put("Inlet_Velocity_2", 10.0); // value for inlet 2

        configMap.put("Outlet_Pressure_1", 0.0); // value for outlet 1
        configMap.put("Outlet_Pressure_2", 0.0); // value for outlet 2

        configMap.put("Inlet_Hydraulic_Diameter_1", 0.1);
        configMap.put("Inlet_Hydraulic_Diameter_2", 0.05);

        configMap.put("Outlet_Hydraulic_Diameter_1", 0.1);
        configMap.put("Outlet_Hydraulic_Diameter_2", 0.05);
    }

    // ==========================================================
    // DYNAMIC PARAMETER CREATION
    // ==========================================================
    private void createDynamicParameters(Simulation sim, GeometryPart part) {
        sim.println("--- Mapping Configuration to Geometry ---");

        for (PartSurface surf : part.getPartSurfaces()) {
            String name = surf.getPresentationName().toLowerCase();

            // ----------------------------------------------------
            // CASE: INLET
            // ----------------------------------------------------
            if (name.contains("inlet") && !name.contains("wall")) {
                int id = extractIDFromName(name);
                String key = "Inlet_Velocity_" + id;
                String dhKey = "Inlet_Hydraulic_Diameter_" + id;

                // CHECK: Do we have a pre-known value for this ID?
                if (configMap.containsKey(key)) {
                    double userValue = configMap.get(key);
                    createOrGetParameter(sim, key, userValue, "m/s");
                    sim.println("   -> Matched " + surf.getPresentationName() + " with configured value: " + userValue);
                } else {
                    // forgot to define this ID
                    //TODO: this is not safe! needs better handling.
                    sim.println("   WARNING: Found " + name + " (ID:" + id + ") but no value defined in configureInputs()!");
                    sim.println("            Using safety default: 1.0 m/s");
                    createOrGetParameter(sim, key, 1.0, "m/s");
                }

                // TODO: handle InletHydraulic Diameter with the same logic
                double dhVal = configMap.getOrDefault(dhKey, 0.08);
                createOrGetParameter(sim, dhKey, dhVal, "m");

            } else if (name.contains("outlet") && !name.contains("wall")) {
                int id = extractIDFromName(name);
                String key = "Outlet_Pressure_" + id;
                String dhKey = "Outlet_Hydraulic_Diameter_" + id;

                if (configMap.containsKey(key)) {
                    double userValue = configMap.get(key);
                    createOrGetParameter(sim, key, userValue, "Pa");
                    sim.println("   -> Matched " + name + " with configured value: " + userValue);
                } else {
                    //TODO: this is not safe! needs better handling.
                    sim.println("   WARNING: Found " + name + " (ID:" + id + ") but no value defined!");
                    createOrGetParameter(sim, key, 0.0, "Pa");
                }

                // TODO: handle Outlet Hydraulic Diameter with the same logic
                double dhVal = configMap.getOrDefault(dhKey, 0.08);
                createOrGetParameter(sim, dhKey, dhVal, "m");
            }

        }

    }

    // HELPER: EXTRACT ID
    private int extractIDFromName(String name) {
        String digit = name.replaceAll("\\D+", "");
        return digit.isEmpty() ? 1: Integer.parseInt(digit);
    }


    // ----------------------------------------------------------
    // HELPER: PARAMETER
    // ----------------------------------------------------------
    private void createGlobalParameters(Simulation sim) {

        createOrGetParameter(sim, "Base_Size", 0.011, "m");
        createOrGetParameter(sim, "Argon_Density", 1.633, "kg/m^3");
        createOrGetParameter(sim, "Argon_Viscosity", 2.23e-5, "Pa-s");
       // createOrGetParameter(sim, "Inlet_Velocity", 20.0, "m/s");
       // createOrGetParameter(sim, "Outlet_Pressure", 0.0, "Pa");
        createOrGetParameter(sim, "Turbulence_Intensity", 0.04, "");
        createOrGetParameter(sim, "Num_Prism_Layers", 10, "");
        createOrGetParameter(sim, "Prism_Layer_Stretching", 1.15, "");
        createOrGetParameter(sim, "Prism_Layer_Thickness", 0.006, "m");
        createOrGetParameter(sim, "Mesh_Volume_Growth_Rate", 1.0, "");

        // Parameters for initial conditions
        createOrGetParameter(sim, "Initial_Pressure", 0.0, "Pa");
        createOrGetParameter(sim, "Initial_Turbulence_Length_Scale", 0.01, "m");

        //TODO: This must me extracted  from inlet geometry
        //createOrGetParameter(sim, "Hydraulic_Diameter", 0.08, "m");
    }

    private void createOrGetParameter(Simulation sim, String name, double value, String unitString) {
        GlobalParameterManager gpm = sim.get(GlobalParameterManager.class);
        ScalarGlobalParameter param;

        if (gpm.has(name)) {
            param = (ScalarGlobalParameter) gpm.getObject(name);
            sim.println("Updated: " + name);
        } else {
            param = (ScalarGlobalParameter) gpm.createGlobalParameter(ScalarGlobalParameter.class, name);
            sim.println("Created: " + name);
        }

        if (unitString != null && !unitString.isEmpty()) {
            Units foundUnit = null;
            for (Units u : sim.getUnitsManager().getObjects()) {
                if (u.getPresentationName().equals(unitString)) {
                    foundUnit = u;
                    break;
                }
            }

            if (foundUnit != null) {
                param.setDimensions(foundUnit.getDimensions());
                param.getQuantity().setUnits(foundUnit);
                param.getQuantity().setValueAndUnits(value, foundUnit);
            } else {
                param.getQuantity().setValue(value);
            }
        } else {
            // Case: Explicitly Dimensionless
            param.getQuantity().setValue(value);
        }
    }

    private ScalarGlobalParameter getParamByName(Simulation sim, String paramName) {
        GlobalParameterManager gpm = sim.get(GlobalParameterManager.class);

        // Safety Check
        if (gpm.has(paramName)) {
            return (ScalarGlobalParameter) gpm.getObject(paramName);
        }

        // If not found, log a warning and return null
        sim.println("Warning: Global Parameter '" + paramName + "' was not found.");
        return null;
    }

    private void linkMaterialProperty(Gas gas, String propName, String paramName, Simulation sim) {
        MaterialProperty prop = (MaterialProperty) gas.getMaterialProperties().getObject(propName);
        ConstantMaterialPropertyMethod method = (ConstantMaterialPropertyMethod) prop.getMethod();
        ScalarGlobalParameter param = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject(paramName);
        method.getQuantity().setDefinition(param);
    }

    // ==========================================================
    // INITIAL CONDITION SETUP
    // ==========================================================
    private void setupInitialConditions(Simulation sim) {
        sim.println("--- Setting Initial Conditions ---");

        if (sim.getContinuumManager().getObjects().isEmpty()) return;
        PhysicsContinuum physics = (PhysicsContinuum) sim.getContinuumManager().getObjects().iterator().next();

        sim.println("   -> Applying conditions to: " + physics.getPresentationName());

        // set static pressure
        InitialPressureProfile pressProfile = physics.getInitialConditions().get(InitialPressureProfile.class);
        ScalarGlobalParameter initPressParam = getParamByName(sim, "Initial_Pressure");

        if (initPressParam != null) {
            ((ConstantScalarProfileMethod) pressProfile.getMethod()).getQuantity().setDefinition(initPressParam);
            sim.println("      -> Initial Pressure linked.");
        }

        // link turbulence
        try {
            physics.getInitialConditions().get(KeTurbSpecOption.class).setSelected(KeTurbSpecOption.Type.INTENSITY_LENGTH_SCALE);

            ScalarGlobalParameter turbParam = getParamByName(sim, "Turbulence_Intensity");
            if (turbParam !=null) {
                TurbulenceIntensityProfile tiProfile = physics.getInitialConditions().get(TurbulenceIntensityProfile.class);
                ((ConstantScalarProfileMethod) tiProfile.getMethod()).getQuantity().setDefinition(turbParam);
            }

            // link length scale
            ScalarGlobalParameter lenParam = getParamByName(sim, "Initial_Turbulence_Length_Scale");
            if (lenParam != null) {
                TurbulentLengthScaleProfile tlsProfile = physics.getInitialConditions().get(TurbulentLengthScaleProfile.class);
                ((ConstantScalarProfileMethod) tlsProfile.getMethod()).getQuantity().setDefinition(lenParam);
            }
            sim.println("      -> Turbulence Initial Conditions set.");

        } catch (Exception e) {
            sim.println("      -> Warning: Turbulence settings skipped (Model mismatch).");
        }
    }

    // ==========================================================
    // GEOMETRY HELPER
    // ==========================================================
    private GeometryPart createPartsGeneric(Simulation sim) {
        if (!sim.getGeometryPartManager().getObjects().isEmpty()) {
            GeometryPart existingPart = sim.getGeometryPartManager().getObjects().iterator().next();
            sim.println("Info: Part already exists (" + existingPart.getPresentationName() + "). Skipping creation.");
            return existingPart;
        }

        SolidModelManager mgr = sim.get(SolidModelManager.class);
        if (mgr.getObjects().isEmpty()) {
            sim.println("Error: No 3D-CAD Models found.");
            return null;
        }
        CadModel cadModel = ((CadModel) sim.get(SolidModelManager.class).getObjects().iterator().next());
        Collection<Body> allBodies = cadModel.getBodyManager().getObjects();

        if (allBodies.isEmpty()) return null;

        sim.println("Creating parts from CAD: " + cadModel.getPresentationName());

        //star.cadmodeler.Body cadModelerBody = (star.cadmodeler.Body) cadModel.getBodyManager().getObjects().iterator().next();
        cadModel.createParts(allBodies, new ArrayList<>(Collections.emptyList()), true,
                false, 1, false, false,
                3, "SharpEdges", 30.0, 2, true,
                1.0E-5, false);

        if (!sim.getGeometryPartManager().getObjects().isEmpty()) {
            return sim.getGeometryPartManager().getObjects().iterator().next();
        }
        return null;
    }

    // ==========================================================
    // PHYSICS HELPER
    // ==========================================================
    private void setupPhysicsAndMaterials(Simulation sim) {
        sim.println("--- Setting up Physics ---");

        PhysicsContinuum physics;
        try {
            physics = (PhysicsContinuum) sim.getContinuumManager().getContinuum("Argon Physics");
        } catch (Exception e) {
            physics = sim.getContinuumManager().createContinuum(PhysicsContinuum.class);
            physics.setPresentationName("Argon Physics");

            physics.enable(ThreeDimensionalModel.class);
            physics.enable(SteadyModel.class);
            physics.enable(SingleComponentGasModel.class);
            physics.enable(CoupledFlowModel.class);
            physics.enable(ConstantDensityModel.class);
            physics.enable(TurbulentModel.class);
            physics.enable(RansTurbulenceModel.class);
            physics.enable(KEpsilonTurbulence.class);
            physics.enable(LienKeTurbModel.class);
            physics.enable(KeLowYplusWallTreatment.class);

            // TODO: not sure if this is necessary
            physics.enable(SolutionInterpolationModel.class);

            // CONFIGURE ARGON
            // ------------------------------------------------
            SingleComponentGasModel gasModel = physics.getModelManager().getModel(SingleComponentGasModel.class);
            Gas argonGas = (Gas) gasModel.getMaterial();
            argonGas.setPresentationName("Argon");

            // Link Parameters
            linkMaterialProperty(argonGas, "Density", "Argon_Density", sim);
            linkMaterialProperty(argonGas, "Dynamic Viscosity", "Argon_Viscosity", sim);

            sim.println("--- Success: Physics Setup Complete ---");
        }
    }

    // ==========================================================
    // REGION & BOUNDARY HELPER
    // ==========================================================
    private void setupRegionAndBoundaries(Simulation sim, GeometryPart part) {
        sim.println("--- Setting up Regions & Boundaries ---");

        String partName = part.getPresentationName();

        if (sim.getRegionManager().has(partName)) {
            sim.println("Info: Region '" + partName + "' already exists. Skipping creation.");
            return;
        }

        // Assign Part
        sim.println("Creating new Region for part: " + partName);
        sim.getRegionManager().newRegionsFromParts(new ArrayList<>(List.<GeometryPart>of(part)), "OneRegionPerPart",
                null, "OneBoundaryPerPartSurface", null, RegionManager.CreateInterfaceMode.BOUNDARY,
                "OneEdgeBoundaryPerPart", null);

        sim.println("--- Classifying Boundaries ---");

        Region region = sim.getRegionManager().getRegion(partName);

        // Shared Turbulence Parameter
        ScalarGlobalParameter globalTurb = getParamByName(sim, "Turbulence_Intensity");

        for (Boundary b : region.getBoundaryManager().getBoundaries()) {
            String name = b.getPresentationName().toLowerCase();

            if (name.contains("inlet") && !name.contains("wall")) {
                int id = extractIDFromName(name);

                String velName = "Inlet_Velocity_" + id;
                String dhName = "Inlet_Hydraulic_Diameter_" + id;

                ScalarGlobalParameter velParam = getParamByName(sim, velName);
                ScalarGlobalParameter dhParam = getParamByName(sim, dhName);

                //ScalarGlobalParameter velParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Inlet_Velocity");
                //ScalarGlobalParameter turbParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Turbulence_Intensity");
                //ScalarGlobalParameter dhParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Hydraulic_Diameter");

                // Safety check: verify we actually found them
                if (velParam != null) {
                    configureInlet(sim, b, velParam, globalTurb, dhParam);
                    sim.println("   -> Configured " + b.getPresentationName() + " using " + velName);
                } else {
                    sim.println("   ERROR: Could not find parameter " + velName + " for boundary " + b.getPresentationName());
                }

            } else if (name.contains("outlet") && !name.contains("wall")) {
                int id = extractIDFromName(name);

                String pressName = "Outlet_Pressure_" + id;
                String dhName = "Outlet_Hydraulic_Diameter_" + id;

                ScalarGlobalParameter pressParam = getParamByName(sim, pressName);
                ScalarGlobalParameter dhParam =  getParamByName(sim, dhName);

                //ScalarGlobalParameter pressureParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Outlet_Pressure");
                //ScalarGlobalParameter turbParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Turbulence_Intensity");
                //ScalarGlobalParameter dhParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Hydraulic_Diameter");

                if (pressParam != null) {
                    configureOutlet(sim, b, pressParam, globalTurb, dhParam);
                    sim.println("   -> Configured " + b.getPresentationName() + " using " + pressName);
                } else {
                    sim.println("   ERROR: Could not find parameter " + pressName + " for boundary " + b.getPresentationName());
                }

            } else {
                b.setBoundaryType(WallBoundary.class);
            }
        }
    }

    // ==========================================================
    // CONFIGURE INLET HELPER
    // ==========================================================
    private void configureInlet(Simulation sim, Boundary boundary, ScalarGlobalParameter velocityParam,
                                ScalarGlobalParameter turbParam, ScalarGlobalParameter dhParam) {
        if (!(boundary.getBoundaryType().getClass().equals(InletBoundary.class))) {
            boundary.setBoundaryType(InletBoundary.class);
        }

        if (velocityParam != null) {
            VelocityMagnitudeProfile velProfile = boundary.getValues().get(VelocityMagnitudeProfile.class);
            ConstantScalarProfileMethod velMethod = (ConstantScalarProfileMethod) velProfile.getMethod();
            velMethod.getQuantity().setDefinition(velocityParam);
        }

        // configure turbulence
        // TODO: make it more general! currently it only works for K-Epsilon model
        boundary.getConditions().get(KeTurbSpecOption.class).setSelected(KeTurbSpecOption.Type.INTENSITY_LENGTH_SCALE);

        if (turbParam != null) {
            TurbulenceIntensityProfile tiProfile = boundary.getValues().get(TurbulenceIntensityProfile.class);
            ConstantScalarProfileMethod tiMethod = (ConstantScalarProfileMethod) tiProfile.getMethod();
            tiMethod.getQuantity().setDefinition(turbParam);
        }

        if (dhParam != null) {
            double dhValue = dhParam.getQuantity().getSIValue();
            double lengthScale = 0.07 * dhValue;
            TurbulentLengthScaleProfile tlProfile = boundary.getValues().get(TurbulentLengthScaleProfile.class);
            ConstantScalarProfileMethod tlMethod = (ConstantScalarProfileMethod) tlProfile.getMethod();
            tlMethod.getQuantity().setValue(lengthScale);
        }
    }

    // ==========================================================
    // CONFIGURE OUTLET HELPER
    // ==========================================================
    private void configureOutlet(Simulation sim, Boundary boundary, ScalarGlobalParameter pressureParam,
                                 ScalarGlobalParameter turbParam, ScalarGlobalParameter dhParam) {
        if (!boundary.getBoundaryType().getClass().equals(PressureBoundary.class)) {
            boundary.setBoundaryType(PressureBoundary.class);
        }

        if (pressureParam != null) {
            StaticPressureProfile pProfile = boundary.getValues().get(StaticPressureProfile.class);
            ConstantScalarProfileMethod pMethod = (ConstantScalarProfileMethod) pProfile.getMethod();
            pMethod.getQuantity().setDefinition(pressureParam);
        }

        // configure backflow turbulence
        // TODO: make it more general! currently it only works for K-Epsilon model
        boundary.getConditions().get(KeTurbSpecOption.class).setSelected(KeTurbSpecOption.Type.INTENSITY_LENGTH_SCALE);

        if (turbParam != null) {
            TurbulenceIntensityProfile tiProfile = boundary.getValues().get(TurbulenceIntensityProfile.class);
            ConstantScalarProfileMethod tiMethod = (ConstantScalarProfileMethod) tiProfile.getMethod();
            tiMethod.getQuantity().setDefinition(turbParam);
        }

        if (dhParam != null) {
            double dhValue = dhParam.getQuantity().getSIValue();
            double lengthScale = 0.07 * dhValue;
            TurbulentLengthScaleProfile tlProfile = boundary.getValues().get(TurbulentLengthScaleProfile.class);
            ConstantScalarProfileMethod tlMethod = (ConstantScalarProfileMethod) tlProfile.getMethod();
            tlMethod.getQuantity().setValue(lengthScale);
        }
    }

    // ==========================================================
    // MESH HELPER
    // ==========================================================
    private void setupMesh(Simulation sim, GeometryPart part) {
        sim.println("--- Setting up Mesh Operation ---");

        MeshOperationManager meshManager = sim.get(MeshOperationManager.class);
        AutoMeshOperation meshOp = null;

        // "star.bodyfittedmesher.AdvancingLayerAutoMesher"
        String[] mesherNames = new String[] {
                "star.resurfacer.ResurfacerAutoMesher",
                "star.dualmesher.DualAutoMesher",
                "star.prismmesher.PrismAutoMesher"
        };

        if (meshManager.has("Automated Mesh")) {
            meshOp = (AutoMeshOperation) meshManager.getObject("Automated Mesh");
            sim.println("Info: 'Automated Mesh' already exists.");
        } else {
            meshOp = meshManager.createAutoMeshOperation(new StringVector(mesherNames),
                    new ArrayList<>(Arrays.<GeometryPart>asList(part)));

            meshOp.setPresentationName("Automated Mesh");
            sim.println("Created: Automated Mesh (Meshers pre-selected)");
        }

        if (meshOp == null) return;

        ScalarGlobalParameter baseSizeParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Base_Size");
        if (baseSizeParam !=null) {
            meshOp.getDefaultValues().get(BaseSize.class).setDefinition(baseSizeParam);

            // CONFIGURE PRISM LAYERS
            meshOp.getDefaultValues().get(NumPrismLayers.class).getNumLayersValue().getQuantity().
                    setDefinition(getParamByName(sim, "Num_Prism_Layers"));

            meshOp.getDefaultValues().get(PrismLayerStretching.class).getStretchingQuantity().
                    setDefinition(getParamByName(sim, "Prism_Layer_Stretching"));

            PrismThickness plt = meshOp.getDefaultValues().get(PrismThickness.class);
            plt.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.ABSOLUTE);
            plt.getAbsoluteSizeValue().setDefinition(getParamByName(sim, "Prism_Layer_Thickness"));

            // SET VOLUME GROWTH RATE
            PartsTetPolyGrowthRate ptpgr = meshOp.getDefaultValues().get(PartsTetPolyGrowthRate.class);
            ScalarGlobalParameter growthParam = getParamByName(sim, "Mesh_Volume_Growth_Rate");
            if (growthParam != null) {
                double growthRateValue = growthParam.getQuantity().getSIValue();
                ptpgr.setGrowthRate(growthRateValue);
            }


        }

        sim.println("--- Mesh Setup Complete ---");
    }

    // ==========================================================
    // MESH SURFACE CONTROL HELPER
    // ==========================================================
        private void addControlToAllSurfaces(Simulation sim, AutoMeshOperation meshOp, GeometryPart part) {
            sim.println("--- Adding Surface Custom Control ---");

            Collection<PartSurface> allSurface = part.getPartSurfaces();

            // Get custom control manager
            CustomMeshControlManager controlManager = meshOp.getCustomMeshControls();

            SurfaceCustomMeshControl surfControl = null;

            // Create or retrieve custom control
            String controlName = "Control_All_Surfaces";

            if (controlManager.has(controlName)) {
                surfControl = (SurfaceCustomMeshControl) controlManager.getObject(controlName);
                sim.println("Info: Custom control '" + controlName + "' already exists.");
            } else {
                surfControl = controlManager.createSurfaceControl();
                surfControl.setPresentationName(controlName);

                surfControl.getGeometryObjects().setQuery(null);
                surfControl.getGeometryObjects().setObjects(allSurface);


                sim.println("Created: " + controlName + " (Applied to part: " + part.getPresentationName() + ")");
            }

            // --- CONFIGURE TARGET SIZE (50% Relative) ---
            surfControl.getCustomConditions().get(PartsTargetSurfaceSizeOption.class).setSelected(PartsTargetSurfaceSizeOption.Type.CUSTOM);
            PartsTargetSurfaceSize surfSize = surfControl.getCustomValues().get(PartsTargetSurfaceSize.class);
            surfSize.getRelativeSizeScalar().setValue(50.0);

            sim.println("   -> Configured Custom Control: " + controlName + " (Target: 50%)");
        }

}