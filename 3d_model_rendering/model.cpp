#include "model.h"


Model::Model(QString& path)
    : indexBuf(QOpenGLBuffer::IndexBuffer)
{
    initializeOpenGLFunctions();
    loadModel(path);

    qDebug() << "model loaded successfully";
    arrayBuf.create();
    indexBuf.create();

    setup();
}

void Model::setup(){
    arrayBuf.bind();
    arrayBuf.allocate(modelVertices.constData(), modelVertices.size() * sizeof(Vertex));

    indexBuf.bind();
    indexBuf.allocate(modelIndices.constData(), modelIndices.size() * sizeof(GLuint));
}

void Model::Draw(QOpenGLShaderProgram * program, GLuint mode){
    program->bind();
    arrayBuf.bind();
    indexBuf.bind();

    int vertexLocation = program->attributeLocation("a_position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, sizeof(Vertex));

	int normalLocation = program->attributeLocation("a_normal");
	program->enableAttributeArray(normalLocation);
	program->setAttributeBuffer(normalLocation, GL_FLOAT, 3, 3, sizeof(Vertex));

    if (mode == 0){
        glDrawElements(GL_TRIANGLES, modelIndices.size(), GL_UNSIGNED_INT, 0);
    }
    else{
        glDrawElements(GL_POINTS, modelIndices.size(), GL_UNSIGNED_INT, 0);
    }
    program->release();
}


void Model::loadModel(QString &path)
{
    const std::string filepath = QS2S(path);
    pcl::PolygonMesh mesh;
	pcl::io::loadPLYFile(filepath, mesh);

	std::cout << mesh.polygons.size() << std::endl;
	
	for (auto it = mesh.polygons.begin(); it != mesh.polygons.end(); it++) {
		modelIndices.push_back(it->vertices[0]);
		modelIndices.push_back(it->vertices[1]);
		modelIndices.push_back(it->vertices[2]);
	}

	
	auto cloud = mesh.cloud;
	int size = cloud.width;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
	// auto cloud_with_normals = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
	pcl::fromPCLPointCloud2(mesh.cloud, *cloudWithNormals);

	modelVertices.resize(cloudWithNormals->size());

	for (int i = 0; i < cloudWithNormals->size(); i++) {
		modelVertices[i].Position = QVector3D{ cloudWithNormals->points[i].x, cloudWithNormals->points[i].y, cloudWithNormals->points[i].z};
		modelVertices[i].Normal = QVector3D{ cloudWithNormals->points[i].normal_x, cloudWithNormals->points[i].normal_y, cloudWithNormals->points[i].normal_z };
	}
	/*for (int i = 0; i < vertices.size(); i++) {
		cout << vertices[i].position[0] << ' ' << vertices[i].position[1] << ' ' << vertices[i].position[2] << endl;
		cout << vertices[i].normal[0] << ' ' << vertices[i].normal[1] << ' ' << vertices[i].normal[2] << endl;
	}*/
	
}
