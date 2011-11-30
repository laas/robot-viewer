
class Vbo(object):
    """
    """

    def __init__(self, shape):
        """

        Arguments:
        - `shape`:
        """
        self.ver_vboId  = -1
        self.nor_vboId  = -1
        self.tri_idx_vboId  = -1
        self.quad_idx_vboId  = -1
        self.poly_idx_vboIds  = []

        self.verts = shape.geometry.coord.point

        if not shape.geometry.normal.vector[:] or not shape.geometry.tri_idxs[:]:
            shape.geometry.compute_normals()

        self.normal = shape.geometry.normal.vector
        self.tri_idxs  = shape.geometry.tri_idxs
        self.quad_idxs = shape.geometry.quad_idxs
        self.poly_idxs = shape.geometry.poly_idxs
        self.tri_count  = shape.geometry.tri_count
        self.quad_count = shape.geometry.tri_count

        logger.debug("Computing normals")

        logger.debug("Loading to GPUs")
        self.load_gpu(shape)

    def __del__(self):
        for vboid in [self.tri_idx_vboId, self.quad_idx_vboId] + self.poly_idx_vboIds:
            glDeleteBuffersARB(1, vboid)
        object.__del__(self)


    def load_gpu(self, shape):
        logger.debug("Creating VBO for shape %s"%shape.name)
        self.tri_count = len(self.tri_idxs)
        self.quad_count = len(self.quad_idxs)
        self.ver_vboId = int(glGenBuffersARB(1))
        logger.debug("Populating VBO for vertices: vboID %d"%self.ver_vboId)
        glBindBufferARB( GL_ARRAY_BUFFER_ARB,self.ver_vboId );
        glBufferDataARB( GL_ARRAY_BUFFER_ARB,
                             numpy.array (self.verts, dtype=numpy.float32),
                             GL_STATIC_DRAW_ARB );
        glBindBufferARB( GL_ARRAY_BUFFER_ARB,0 );
        logger.debug("Generated VBO for vertices: vboID %d"%self.ver_vboId)

        self.nor_vboId = int(glGenBuffersARB(1))
        logger.debug("Populating VBO for normals: vboID %d"%self.nor_vboId)
        glBindBufferARB( GL_ARRAY_BUFFER_ARB,self.nor_vboId );
        glBufferDataARB( GL_ARRAY_BUFFER_ARB,
                             numpy.array (self.normal, dtype=numpy.float32),
                             GL_STATIC_DRAW_ARB );
        glBindBufferARB( GL_ARRAY_BUFFER_ARB,0 );
        logger.debug("Generated VBO for normals: vboID %d"%self.nor_vboId)

        self.tri_idx_vboId = int(glGenBuffersARB(1))
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB,self.tri_idx_vboId );
        glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB,
                             numpy.array (self.tri_idxs, dtype=numpy.uint16),
                             GL_STATIC_DRAW_ARB );
        logger.debug("Generated VBO for triangle indices: vboID %d"%
                     self.tri_idx_vboId)
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB,0 );
        logger.debug("Finished creating VBO for shape %s"%shape.name)

        self.quad_idx_vboId = int(glGenBuffersARB(1))
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB,self.quad_idx_vboId );
        glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB,
                             numpy.array (self.quad_idxs, dtype=numpy.uint16),
                             GL_STATIC_DRAW_ARB );
        logger.debug("Generated VBO for quadangle indices: vboID %d"%
                     self.quad_idx_vboId)
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB,0 );

        for i,poly in enumerate(self.poly_idxs):
            poly_idx_vboId = int(glGenBuffersARB(1))
            self.poly_idx_vboIds.append(poly_idx_vboId)
            glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, poly_idx_vboId );
            glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB,
                             numpy.array (poly, dtype=numpy.uint16),
                             GL_STATIC_DRAW_ARB );
            logger.debug("Generated VBO for poly indices: vboID %d"%
                         poly_idx_vboId)
            glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB,0 );

        logger.debug("Finished creating VBO for shape %s"%shape.name)


    def __str__(self):
        """
        """
        s="[Vbo instance:\n"
        s+="ver_vboId\t=%d\n"%self.ver_vboId
        s+="nor_vboId\t=%d\n"%self.nor_vboId
        s+="tri_idx_vboId\t=%d\n"%self.tri_idx_vboId
        s+="quad_idx_vboId\t=%d\n"%self.quad_idx_vboId

        s+="len (_verts)\t=%d\n"%(len(self.verts))
        s+="len (_norms)\t=%d\n"%(len(self.normal))
        s+="len (_idxs)\t=%d\n"%(len(self.tri_idxs))
        s+="]"
        return s
