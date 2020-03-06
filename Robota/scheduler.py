import subprocess
import os
import numpy

currentPath = os.path.dirname(os.path.abspath(__file__))

# # Type A simulation
# baseDatadir = "./data/typeA"
# oscillation = True
# obstacleAvoidance = False
# dampings = numpy.arange(start=0.0, stop=6.0, step=1)
# kappa = numpy.arange(start=4.0, stop=39.0, step=1.0)
# maxNumIterations = 20
# for damping in dampings:
#     for k in kappa:
#         savDir = "{}_{}".format(damping, k)
#         savDir = os.path.join(baseDatadir, savDir)
#         runArgs = "--run args k={} kr=0.19 damp={} rts={} collision={} iterations={} savDir={}".format(
#             k, damping, oscillation, obstacleAvoidance, maxNumIterations, savDir
#         ).lower()
#         cmd = ["/opt/processing-3.3.6/processing-java", "--sketch={}".format(currentPath), runArgs]
#         subprocess.call(' '.join(cmd), shell=True)

# Type B simulation
# baseDatadir = "./data/typeB"
# oscillation = True
# obstacleAvoidance = True
# dampings = numpy.arange(start=0.0, stop=6.0, step=1)
# kappa = numpy.arange(start=4.0, stop=39.0, step=1.0)
# maxNumIterations = 20
# for damping in dampings:
#     for k in kappa:
#         savDir = "{}_{}".format(damping, k)
#         savDir = os.path.join(baseDatadir, savDir)
#         runArgs = "--run args k={} kr=0.19 damp={} rts={} collision={} iterations={} savDir={}".format(
#             k, damping, oscillation, obstacleAvoidance, maxNumIterations, savDir
#         ).lower()
#         cmd = ["/opt/processing-3.3.6/processing-java", "--sketch={}".format(currentPath), runArgs]
#         subprocess.call(' '.join(cmd), shell=True)

# Type C-alpha
baseDatadir = "./data/typeCAlpha"
oscillation = False
obstacleAvoidance = True
dampings = numpy.arange(start=0.0, stop=6.0, step=1)
kappa = numpy.arange(start=4.0, stop=39.0, step=1.0)
maxNumIterations = 10
for damping in dampings:
    for k in kappa:
        savDir = "{}_{}".format(damping, k)
        savDir = os.path.join(baseDatadir, savDir)
        runArgs = "--run args k={} kr=0.19 damp={} rts={} collision={} iterations={} savDir={}".format(
            k, damping, oscillation, obstacleAvoidance, maxNumIterations, savDir
        ).lower()
        cmd = ["/opt/processing-3.3.6/processing-java", "--sketch={}".format(currentPath), runArgs]
        subprocess.call(' '.join(cmd), shell=True)
