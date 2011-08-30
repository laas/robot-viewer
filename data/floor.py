lines=[]
N=10
L=0.5*N

for i in range(-N,N+1):
    lines.append([i*0.5, L,0.005])
    lines.append([i*0.5,-L,0.005])
    lines.append([i*0.5,-L,0.005])
    lines.append([i*0.5, L,0.005])

    lines.append([L    , i*0.5,0.005])
    lines.append([-L   , i*0.5,0.005] )
    lines.append([-L   , i*0.5,0.005] )
    lines.append([L    , i*0.5,0.005])

glMaterialfv(GL_FRONT_AND_BACK,  GL_AMBIENT_AND_DIFFUSE, [.1,.1,.1,1])
glMaterialfv(GL_FRONT_AND_BACK,  GL_SPECULAR           , [1,1,1,1])
glMaterialfv(GL_FRONT_AND_BACK,  GL_EMISSION           , [0,0,0,1])

glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 6)
glBegin(GL_LINES)

for point in lines:
    glVertex3f(point[0],point[1],point[2])
glEnd()
