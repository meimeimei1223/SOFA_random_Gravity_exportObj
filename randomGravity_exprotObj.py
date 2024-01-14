# Required import for python
import Sofa
from Sofa.Helper import msg_info
import random
import os


Root_path = "mesh/"
Root_model_save_path = "mesh/generated_mesh/"

if not os.path.exists(Root_model_save_path):
    os.makedirs(Root_model_save_path)
    
#liver_obj_path = Root_path + "liver2.obj"
#liver_gmsh_path = Root_path + "liverstl.msh"
portal_path = Root_path + "liver.obj"
vein_path =  Root_path +  "liver.obj"
mask_path = Root_path + "liver.obj"
res_path = Root_path + "liver.obj"
tumor_path = Root_path + "liver.obj"
gb_path = Root_path +  "liver.obj"


list = ["liver", "portal", "vein", "mask", "res", "tumor", "gb"]

# ディレクトリが存在しない場合は作成
for list in list:
 if not os.path.exists(Root_model_save_path + list):
    os.makedirs(Root_model_save_path + list)

liver_model_save_path = Root_model_save_path + "liver/liver_obj"
portal_model_save_path = Root_model_save_path + "portal/liver_obj"
vein_model_save_path = Root_model_save_path + "vein/vein_obj"
mask_model_save_path = Root_model_save_path + "mask/mask_obj"
tumor_model_save_path = Root_model_save_path + "tumor/tumor_obj"
res_model_save_path = Root_model_save_path + "res/res_obj"
gb_model_save_path = Root_model_save_path + "gb/gb_obj"



# Choose in your script to activate or not the GUI
USE_GUI = True

class ManageGravity(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.CFF = kwargs.get("ForceField")
        self.rootNode = kwargs.get("rootNode")
      
        self.sum = 0
        self.iteration = 0
        print(self.iteration)
        random.seed(0) 
        
    def onAnimateBeginEvent(self, event):
        dt = self.rootNode.findData('dt').value
        self.sum += dt 
        
        if self.iteration >= 20:
            Sofa.Gui.GUIManager.closeGUI()
            
        
        if self.sum >= 10:
            self.iteration += 1
            self.sum = 0
            print(f"Second passed: {self.iteration}")
            gravity = self.rootNode.findData('gravity').value
            gravity = [random.uniform(-200, 200) for _ in range(3)]  #gravity = [-x + for x in gravity]
            self.rootNode.findData('gravity').value = gravity
            print(gravity)
               
def main():
    import SofaRuntime
    import Sofa.Gui

    root = Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)

    if not USE_GUI:
        for iteration in range(10):
            Sofa.Simulation.animate(root, root.dt.value)
    else:
        Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
        Sofa.Gui.GUIManager.createGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()


def createScene(root):
    root.gravity=[0, 100, 0]
    root.dt=0.2

    root.addObject("RequiredPlugin", pluginName=[    'Sofa.Component.Collision.Detection.Algorithm',
    'Sofa.Component.Collision.Detection.Intersection',
    'Sofa.Component.Collision.Geometry',
    'Sofa.Component.Collision.Response.Contact',
    'Sofa.Component.Constraint.Projective',
    'Sofa.Component.IO.Mesh',
    'Sofa.Component.LinearSolver.Iterative',
    'Sofa.Component.Mapping.Linear',
    'Sofa.Component.Mass',
    'Sofa.Component.ODESolver.Backward',
    'Sofa.Component.SolidMechanics.FEM.Elastic',
    'Sofa.Component.StateContainer',
    'Sofa.Component.Topology.Container.Dynamic',
    'Sofa.Component.Visual',
    'Sofa.GL.Component.Rendering3D'
    ])

    root.addObject('DefaultAnimationLoop')
    root.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels hideCollisionModels hideMapping hideOptions hideMechanics')
    
    root.addObject('CollisionPipeline', name="CollisionPipeline")
    root.addObject('BruteForceBroadPhase', name="BroadPhase")
    root.addObject('BVHNarrowPhase', name="NarrowPhase")
    root.addObject('DefaultContactManager', name="CollisionResponse", response="PenalityContactForceField")
    root.addObject('DiscreteIntersection')

    root.addObject('MeshOBJLoader', name="LiverSurface", filename="mesh/liver-smooth.obj")
    #root.addObject('MeshObjLoader', name="LiverSurface", filename=liver_obj_path)
    liver = root.addChild('Liver')
    liver.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness=0.1, rayleighMass=0.1)
    #liver.addObject('CGLinearSolver', name="linear_solver", iterations=25, tolerance=1e-09, threshold=1e-09)
    liver.addObject('CGLinearSolver', name="linear_solver", iterations="200", tolerance="1e-09", threshold="1e-09")
    liver.addObject('MeshGmshLoader', name="meshLoader", filename="mesh/liver.msh")
    #liver.addObject('MeshGmshLoader', name="meshLoader", filename=liver_gmsh_path)
    liver.addObject('TetrahedronSetTopologyContainer', name="topo", src="@meshLoader")
    liver.addObject('MechanicalObject', name="dofs", src="@meshLoader")
    liver.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
    liver.addObject('DiagonalMass', name="Mass", massDensity=1.0)
    #liver.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio=0.3, youngModulus=3000, computeGlobalMatrix=False)
    liver.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio="0.4", youngModulus="10000", computeGlobalMatrix="0")
    liver.addObject('FixedConstraint', name="FixedConstraint", indices="11 12 20 25 63")  #100
    liver.addObject('ConstantForceField', name="CFF", totalForce="0 0 0 0 0 0", indices="83") #indices="13"

    visu = liver.addChild('Visu')
    #visu.addObject('OglModel', name="VisualModel", src="@../../LiverSurface",texturename="textures/liver-texture-square.png")
    visu.addObject('OglModel', name="VisualModel", src="@../../LiverSurface",color="0.8 0.6 0.4 0.9")
    visu.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")
    visu.addObject("VisualModelOBJExporter",name="exporter",filename=liver_model_save_path
    ,exportEveryNumberOfSteps=50,exportAtBegin = True) 
    
    root.addObject('MeshObjLoader', name="portal", filename= portal_path)
    visuP = liver.addChild('portal')
    visuP.addObject('OglModel', name="VisualModel", src="@../../portal",color="1  0.2  0.8 0.5" )
    visuP.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")
    visuP.addObject("VisualModelOBJExporter",name="exporterP",filename=portal_model_save_path 
    ,exportEveryNumberOfSteps=50,exportAtBegin = True)
    
    root.addObject('MeshObjLoader', name="vein", filename=vein_path)
    visuV = liver.addChild('vein')
    visuV.addObject('OglModel', name="VisualModel", src="@../../vein",color="0.2  1  1 0.5"  )
    visuV.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")
    visuV.addObject("VisualModelOBJExporter",name="exporterP",filename=vein_model_save_path 
    ,exportEveryNumberOfSteps=50,exportAtBegin = True)
    
    root.addObject('MeshObjLoader', name="mask", filename=mask_path)
    visuM = liver.addChild('mask')
    visuM.addObject('OglModel', name="VisualModel", src="@../../mask",color="1  0.2  0.8 0.5" )
    visuM.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")
    visuM.addObject("VisualModelOBJExporter",name="exporterP",filename=mask_model_save_path 
    ,exportEveryNumberOfSteps=50,exportAtBegin = True)
    
    root.addObject('MeshObjLoader', name="res", filename=res_path)
    visuR = liver.addChild('res')
    visuR.addObject('OglModel', name="VisualModel", src="@../../res",color="0.2  1  0.2 0.5"  )
    visuR.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")
    visuR.addObject("VisualModelOBJExporter",name="exporterP",filename=res_model_save_path 
    ,exportEveryNumberOfSteps=50,exportAtBegin = True)
    
    
    root.addObject('MeshObjLoader', name="tumor", filename=tumor_path)
    visuT = liver.addChild('tumor')
    visuT.addObject('OglModel', name="VisualModel", src="@../../tumor",color="1  0  0.8 0.5"  ) 
    visuT.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")
    visuT.addObject("VisualModelOBJExporter",name="exporterP",filename=tumor_model_save_path 
    ,exportEveryNumberOfSteps=50,exportAtBegin = True)
    
    
    root.addObject('MeshObjLoader', name="gb", filename=gb_path)
    visuG = liver.addChild('gb')
    visuG.addObject('OglModel', name="VisualModel", src="@../../gb",color="0.1  1  0.1 0.3"  )
    visuG.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")
    visuG.addObject("VisualModelOBJExporter",name="exporterP",filename=gb_model_save_path 
    ,exportEveryNumberOfSteps=50,exportAtBegin = True)
    
    

    surf = liver.addChild('Surf')
    surf.addObject('MeshOBJLoader', name='loader', filename="mesh/liver-smooth.obj")
    surf.addObject('MechanicalObject', src='@loader')
    surf.addObject('TriangleCollisionModel')
    surf.addObject('LineCollisionModel')
    surf.addObject('PointCollisionModel')
    surf.addObject('BarycentricMapping')

   
    root.addObject( ManageGravity(name="GravityController", ForceField=liver.CFF, rootNode=root, Visu = visu))

    return root



# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
