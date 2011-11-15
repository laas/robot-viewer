function [nf,nv] = reduce_mesh(verts, faces, r)
p = patch('Faces',faces,'Vertices',verts,'FaceColor','w');
h = axes;
p2 = copyobj(p,h);
[nf,nv] = reducepatch(p2,r);
end
