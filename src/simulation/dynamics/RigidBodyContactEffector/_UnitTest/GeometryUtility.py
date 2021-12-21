import numpy as np
import sys
import copy
import tinyobjloader


class SurfaceGeometry:
    def __init__(self, fileList, resDist):
        self.allObjs = []
        self.discoveredFaces = []
        self.LoadObjects(fileList, resDist)
        self.facesToKnow = 0
        for faceSet in self.discoveredFaces:
            self.facesToKnow += len(faceSet) - 1
        self.facesKnown = 0
        self.facesKnownProp = 0.0



    def LoadObjects(self, fileList, resDist):
        reader = tinyobjloader.ObjReader()
        config = tinyobjloader.ObjReaderConfig()
        config.triangulate = False
        for file, res in zip(fileList, resDist):
            fileVerts = []
            fileNorms = []
            ret = reader.ParseFromFile(file, config)
            if ret == False:
                print("Failed to load : ", file)
                print("Warn:", reader.Warning())
                print("Err:", reader.Error())
                sys.exit(-1)

            if reader.Warning():
                print("Warn:", reader.Warning())

            attrib = reader.GetAttrib()
            shapes = reader.GetShapes()
            for ii in range(0, len(attrib.vertices), 3):
                fileVerts.append(np.array([attrib.vertices[ii], attrib.vertices[ii+1], attrib.vertices[ii+2]]))

            for ii in range(0, len(attrib.normals), 3):
                fileNorms.append(np.array([attrib.normals[ii], attrib.normals[ii+1], attrib.normals[ii+2]]))

            for shape in shapes:
                shapeVerts = []
                shapeVertsInd = []
                shapeVertsIndTrunc = []
                faceNorms = []
                facePoints = []
                edgeIndices = []
                faceIndices = []
                faceDiscovery = []
                centroid = np.array([0.0, 0.0, 0.0])

                for index in shape.mesh.indices:
                    shapeVertsInd.append(index.vertex_index)

                indexOffset = 0
                for faceIt in range(len(shape.mesh.num_face_vertices)):
                    v1 = fileVerts[shapeVertsInd[indexOffset+1]] - fileVerts[shapeVertsInd[indexOffset]]
                    v2 = fileVerts[shapeVertsInd[indexOffset+2]] - fileVerts[shapeVertsInd[indexOffset+1]]
                    faceNormal = np.around(np.cross(v1, v2), 15) + np.array([0.0, 0.0, 0.0])
                    faceNorms.append(faceNormal / np.linalg.norm(faceNormal))
                    faceDiscovery.append(0)

                    pointsOnFace = []
                    for ii in range(shape.mesh.num_face_vertices[faceIt] - 1):
                        edgeIndices.append([shapeVertsInd[indexOffset + ii], shapeVertsInd[indexOffset + ii + 1]])
                        faceIndices.append(faceIt)
                        pointsOnFace.append(shapeVertsInd[indexOffset + ii])

                    edgeIndices.append([shapeVertsInd[indexOffset + shape.mesh.num_face_vertices[faceIt] - 1],
                                        shapeVertsInd[indexOffset]])
                    faceIndices.append(faceIt)
                    pointsOnFace.append(shapeVertsInd[indexOffset + shape.mesh.num_face_vertices[faceIt] - 1])
                    facePoints.append(pointsOnFace.copy())
                    indexOffset += shape.mesh.num_face_vertices[faceIt]

                skipNext = False
                for edgeIt in range(len(edgeIndices)):
                    if skipNext:
                        skipNext = False
                        continue
                    searchEdge = [edgeIndices[edgeIt][1], edgeIndices[edgeIt][0]]
                    for searchIt in range(edgeIt+1, len(edgeIndices)):
                        if edgeIndices[searchIt] == searchEdge:
                            itEdge = edgeIt + 1
                            itSearch = searchIt + 1
                            edgeIndices.insert(itEdge, searchEdge.copy())
                            del edgeIndices[itSearch]
                            faceIndices.insert(itEdge, faceIndices[searchIt])
                            del faceIndices[itSearch]
                            skipNext = True

                [shapeVertsIndTrunc.append(x) for x in shapeVertsInd if x not in shapeVertsIndTrunc]

                for edgeIt in range(len(edgeIndices)):
                    edgeIndices[edgeIt][0] = shapeVertsIndTrunc.index(edgeIndices[edgeIt][0])
                    edgeIndices[edgeIt][1] = shapeVertsIndTrunc.index(edgeIndices[edgeIt][1])

                for faceIt in range(len(facePoints)):
                    for pointIt in range(len(facePoints[faceIt])):
                        facePoints[faceIt][pointIt] = shapeVertsIndTrunc.index(facePoints[faceIt][pointIt])

                for ind in shapeVertsIndTrunc:
                    shapeVerts.append(fileVerts[ind])
                    centroid += fileVerts[ind]
                centroid = centroid / len(shapeVerts)

                maxRadius = 0.0
                xMin = shapeVerts[0][0]
                xMax = shapeVerts[0][0]
                yMin = shapeVerts[0][1]
                yMax = shapeVerts[0][1]
                zMin = shapeVerts[0][2]
                zMax = shapeVerts[0][2]
                for vert in shapeVerts:
                    if np.linalg.norm(vert - centroid) > maxRadius:
                        maxRadius = np.linalg.norm(vert - centroid)
                    if xMin > vert[0]:
                        xMin = vert[0]
                    if xMax < vert[0]:
                        xMax = vert[0]
                    if yMin > vert[1]:
                        yMin = vert[1]
                    if yMax < vert[1]:
                        yMax = vert[1]
                    if zMin > vert[2]:
                        zMin = vert[2]
                    if zMax < vert[2]:
                        zMax = vert[2]

                deltaR_v = np.array([0.5 * (xMax - xMin), 0.5 * (yMax - yMin), 0.5 * (zMax - zMin)])
                projDeltaR_v = []

                for ii in range(len(faceNorms)):
                    dirMax = np.dot(faceNorms[ii], shapeVerts[0] - centroid)

                    for vert in shapeVerts:
                        if dirMax < np.dot(faceNorms[ii], vert - centroid):
                            dirMax = np.dot(faceNorms[ii], vert - centroid)

                    projDeltaR_v.append(float(dirMax))

                faceCentroids = []
                for faceNum in range(len(faceNorms)):
                    faceCentroid = np.array([0.0, 0.0, 0.0])
                    for vertNum in facePoints[faceNum]:
                        faceCentroid += shapeVerts[vertNum]
                    faceCentroid = faceCentroid / len(facePoints[faceNum])
                    faceCentroids.append(faceCentroid.copy())


                shapeData = {'geoCentroid': centroid, 'resDist': res, 'vertices': shapeVerts, 'faceNorms': faceNorms,
                             'edgeIndices': edgeIndices, 'faceIndices': faceIndices, 'boundingRadius': maxRadius,
                             'facePoints': facePoints, 'deltaR_v': deltaR_v, 'projDeltaR_v': projDeltaR_v,
                             'boundingBox': deltaR_v, 'faceCentroids': faceCentroids}
                self.allObjs.append(copy.deepcopy(shapeData))
                self.discoveredFaces.append(faceDiscovery.copy())



class LanderGeometry:
    def __init__(self, fileName, boundingRadius):
        self.boundingRadius = boundingRadius
        self.landerObjs = []
        self.LoadObjects(fileName)

    def LoadObjects(self, fileName):
        reader = tinyobjloader.ObjReader()
        config = tinyobjloader.ObjReaderConfig()
        config.triangulate = False
        fileVerts = []
        ret = reader.ParseFromFile(fileName, config)
        if ret == False:
            print("Failed to load : ", fileName)
            print("Warn:", reader.Warning())
            print("Err:", reader.Error())
            sys.exit(-1)

        if reader.Warning():
            print("Warn:", reader.Warning())

        attrib = reader.GetAttrib()
        shapes = reader.GetShapes()
        for ii in range(0, len(attrib.vertices), 3):
            fileVerts.append(np.array([attrib.vertices[ii], attrib.vertices[ii + 1], attrib.vertices[ii + 2]]))

        for shape in shapes:
            shapeVerts = []
            shapeVertsInd = []
            shapeVertsIndTrunc = []
            faceNorms = []
            facePoints = []
            edgeIndices = []
            faceIndices = []
            centroid = np.array([0.0, 0.0, 0.0])

            for index in shape.mesh.indices:
                shapeVertsInd.append(index.vertex_index)

            indexOffset = 0
            for faceIt in range(len(shape.mesh.num_face_vertices)):
                v1 = fileVerts[shapeVertsInd[indexOffset + 1]] - fileVerts[shapeVertsInd[indexOffset]]
                v2 = fileVerts[shapeVertsInd[indexOffset + 2]] - fileVerts[shapeVertsInd[indexOffset + 1]]
                faceNormal = np.around(np.cross(v1, v2), 15) + np.array([0.0, 0.0, 0.0])
                faceNorms.append(faceNormal / np.linalg.norm(faceNormal))

                pointsOnFace = []
                for ii in range(shape.mesh.num_face_vertices[faceIt] - 1):
                    edgeIndices.append([shapeVertsInd[indexOffset + ii], shapeVertsInd[indexOffset + ii + 1]])
                    faceIndices.append(faceIt)
                    pointsOnFace.append(shapeVertsInd[indexOffset + ii])
                edgeIndices.append([shapeVertsInd[indexOffset + shape.mesh.num_face_vertices[faceIt] - 1],
                                    shapeVertsInd[indexOffset]])
                faceIndices.append(faceIt)
                pointsOnFace.append(shapeVertsInd[indexOffset + shape.mesh.num_face_vertices[faceIt] - 1])
                facePoints.append(pointsOnFace.copy())
                indexOffset += shape.mesh.num_face_vertices[faceIt]

            skipNext = False
            for edgeIt in range(len(edgeIndices)):
                if skipNext:
                    skipNext = False
                    continue
                searchEdge = [edgeIndices[edgeIt][1], edgeIndices[edgeIt][0]]
                for searchIt in range(edgeIt + 1, len(edgeIndices)):
                    if edgeIndices[searchIt] == searchEdge:
                        itEdge = edgeIt + 1
                        itSearch = searchIt + 1
                        edgeIndices.insert(itEdge, searchEdge.copy())
                        del edgeIndices[itSearch]
                        faceIndices.insert(itEdge, faceIndices[searchIt])
                        del faceIndices[itSearch]
                        skipNext = True

            [shapeVertsIndTrunc.append(x) for x in shapeVertsInd if x not in shapeVertsIndTrunc]

            for edgeIt in range(len(edgeIndices)):
                edgeIndices[edgeIt][0] = shapeVertsIndTrunc.index(edgeIndices[edgeIt][0])
                edgeIndices[edgeIt][1] = shapeVertsIndTrunc.index(edgeIndices[edgeIt][1])

            for faceIt in range(len(facePoints)):
                for pointIt in range(len(facePoints[faceIt])):
                    facePoints[faceIt][pointIt] = shapeVertsIndTrunc.index(facePoints[faceIt][pointIt])

            for ind in shapeVertsIndTrunc:
                shapeVerts.append(fileVerts[ind])
                centroid += fileVerts[ind]
            centroid = centroid / len(shapeVerts)



            shapeData = {'geoCentroid': centroid, 'vertices': shapeVerts, 'faceNorms': faceNorms,
                         'edgeIndices': edgeIndices, 'faceIndices': faceIndices, 'facePoints': facePoints}
            self.landerObjs.append(copy.deepcopy(shapeData))



