#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.
    int nextIndex = (int)evaluatedLevels.size();

    int numberOfPoints = evaluatedLevels[0].size() - evaluatedLevels.size();

    if ((int)evaluatedLevels.size() == numControlPoints){
      return;
    }

    vector<Vector2D> nextLevelVector;

    for (int i = 0; i < numberOfPoints; i++){
      Vector2D point = evaluatedLevels[nextIndex-1][i]* (1-t) + evaluatedLevels[nextIndex-1][i+1] * t;
      nextLevelVector.push_back(point);
    }
    
    evaluatedLevels.push_back(nextLevelVector);
  }




  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
    vector<Vector3D> uPoints;
    for (int i = 0; i < controlPoints.size(); i++){
      uPoints.push_back(evaluate1D(controlPoints[i], u));
    }

    return evaluate1D(uPoints, v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.

    

    if (points.size() == 1){
      //returning final point;
      return points[0];
    } else {

      vector<Vector3D> nextOrderOfPoints;
      for (int i = 0; i < points.size()-1; i++){
        Vector3D point = points[i]* (1-t) + points[i+1] * t;
        nextOrderOfPoints.push_back(point);
      }
      return evaluate1D(nextOrderOfPoints, t);
    }

 }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.

    Vector3D sum = Vector3D(0, 0, 0);

    HalfedgeCIter h = halfedge();

    Vector3D commonPoint = h->vertex()->position;

    h = h->twin();

    HalfedgeCIter original = h;


    do{
      
      Vector3D farPoint1 = h->vertex()->position;
      h = h->next()->twin();
      Vector3D farPoint2 = h->vertex()->position;
      Vector3D edgeVector1 = commonPoint - farPoint1;
      Vector3D edgeVector2 = farPoint2 - commonPoint;

      sum += cross(edgeVector1, edgeVector2)/2;
      
    } while(h != original);
    
    return sum.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
    if (e0->isBoundary()){
      return e0;
    }else{

      //Halfedges
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();

      //Vertices
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();

      //Edges
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();

      //Faces
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();


      //Reassignment:

      //Halfedges:

      //For every halfedge: next(), twin(), vertex(), edge(), face() must be set.

      h0->setNeighbors(h1, h3, v3, e0, f0);
      h1->setNeighbors(h2, h7, v2, e2, f0);
      h2->setNeighbors(h0, h8, v0, e3, f0);
      h3->setNeighbors(h4, h0, v2, e0, f1);
      h4->setNeighbors(h5, h9, v3, e4, f1);
      h5->setNeighbors(h3, h6, v1, e1, f1);
      h6->setNeighbors(h6->next(), h5, v2, e1, h6->face());
      h7->setNeighbors(h7->next(), h1, v0, e2, h7->face());
      h8->setNeighbors(h8->next(), h2, v3, e3, h8->face());
      h9->setNeighbors(h9->next(), h4, v1, e4, h9->face());

      //Vertices:

      v0->halfedge() = h2;
      v1->halfedge() = h5;
      v2->halfedge() = h3;
      v3->halfedge() = h0;

      //Edges:

      e0->halfedge() = h0;
      e1->halfedge() = h5;
      e2->halfedge() = h1;
      e3->halfedge() = h2;
      e4->halfedge() = h4;

      //Faces:

      f0->halfedge() = h0;
      f1->halfedge() = h3;

      return e0;
    }
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (e0->isBoundary()){
      //Boundary case ignored;
      return e0->halfedge()->vertex();
    }else{
      //Halfedges
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();

      //New Halfedges needed
      HalfedgeIter h10 = newHalfedge();
      HalfedgeIter h11 = newHalfedge();
      HalfedgeIter h12 = newHalfedge();
      HalfedgeIter h13 = newHalfedge();
      HalfedgeIter h14 = newHalfedge();
      HalfedgeIter h15 = newHalfedge();

      //Vertices
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();

      //New Vertex needed (this is the new vertex to return)
      VertexIter v4 = newVertex();

      

      //Edges
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();

      //New edges needed
      EdgeIter e5 = newEdge();
      EdgeIter e6 = newEdge();
      EdgeIter e7 = newEdge();
      

      //Faces
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();

      //New Faces needed

      FaceIter f2 = newFace();
      FaceIter f3 = newFace();

      //Reassignment:

      //Halfedges:

      //For every halfedge: next(), twin(), vertex(), edge(), face() must be set.

      h0->setNeighbors(h1, h3, v4, e0, f3);
      h1->setNeighbors(h2, h6, v1, e1, f3);
      h2->setNeighbors(h0, h10, v2, e7, f3);
      h3->setNeighbors(h4, h0, v1, e0, f2);
      h4->setNeighbors(h5, h15, v4, e6, f2);
      h5->setNeighbors(h3, h9, v3, e4, f2);
      h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());
      h7->setNeighbors(h7->next(), h11, v0, e2, h7->face());
      h8->setNeighbors(h8->next(), h14, v3, e3, h8->face());
      h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());
      h10->setNeighbors(h11, h2, v4, e7, f0);
      h11->setNeighbors(h12, h7, v2, e2, f0);
      h12->setNeighbors(h10, h13, v0, e5, f0);
      h13->setNeighbors(h14, h12, v4, e5, f1);
      h14->setNeighbors(h15, h8, v0, e3, f1);
      h15->setNeighbors(h13, h4, v3, e6, f1);


      //Vertices:

      v0->halfedge() = h12;
      v1->halfedge() = h3;
      v2->halfedge() = h2;
      v3->halfedge() = h15;
      v4->halfedge() = h0;
      v4->position = 0.5f * v3->position + 0.5f * v2->position;

      //Edges:

      e0->halfedge() = h0;
      e1->halfedge() = h1;
      e2->halfedge() = h7;
      e3->halfedge() = h8;
      e4->halfedge() = h5;
      e5->halfedge() = h12;
      e6->halfedge() = h4;
      e7->halfedge() = h2;

      //Faces:

      f0->halfedge() = h10;
      f1->halfedge() = h13;
      f2->halfedge() = h4;
      f3->halfedge() = h1;

      return v4;
    }
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.
    cout<<"Computing new positions for old vertices" << endl;
    VertexIter currentVert = mesh.verticesBegin();

    while (currentVert != mesh.verticesEnd()){
      
      VertexIter nextVert = currentVert;
      nextVert++;

      currentVert->isNew = false;
      float n = currentVert->degree();
      float u;

      if (n == 3){
        u = 3.F/16.F;
      } else{
        u = 3.F/(8.F*n);
      }

      currentVert->computeCentroid();

      currentVert->newPosition = (1 - n*u) * currentVert->position + u * currentVert->centroid;

      currentVert = nextVert;
    }


    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    cout<<"Computing new positions for edges to be split" << endl;
    EdgeIter currentEdge = mesh.edgesBegin();

    vector<EdgeIter> oldEdges; //caching for next step

    while(currentEdge != mesh.edgesEnd()){
      EdgeIter nextEdge = currentEdge;
      nextEdge++;

      Vector3D p1 = currentEdge->halfedge()->vertex()->position;
      Vector3D p2 = currentEdge->halfedge()->twin()->vertex()->position;
      Vector3D p3 = currentEdge->halfedge()->next()->next()->vertex()->position;
      Vector3D p4 = currentEdge->halfedge()->twin()->next()->next()->vertex()->position;

      currentEdge->newPosition = (3.F/8.F)*(p1+p2) + (1.F/8.F)*(p3+p4);

      oldEdges.push_back(currentEdge);



      currentEdge = nextEdge;
    }

    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flag Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)
    cout<<"Splitting all old edges" << endl;

    for (int i = 0; i< oldEdges.size(); i++){
      Vector3D newPos = oldEdges[i]->newPosition;
      VertexIter newPoint = mesh.splitEdge(oldEdges[i]);

      newPoint->isNew = true;
      HalfedgeIter h = newPoint->halfedge();

      //First neighbor vertex, edge is old
      h->edge()->isNew = false;
      //Second neighbor vertex, edge is new
      h = h->twin()->next();
      h->edge()->isNew = true;
      //Third neigbor vertex, edge is old
      h = h->twin()->next();
      h->edge()->isNew = false;
      //Fourth neigbor vertex, edge is new
      h = h->twin()->next();
      h->edge()->isNew = true;

      newPoint->newPosition = newPos;
    }


    // TODO Now flip any new edge that connects an old and new vertex.
    cout<<"Flipping edges connected to 1 old and 1 new point" << endl;
    currentEdge = mesh.edgesBegin();
    while(currentEdge != mesh.edgesEnd()){
      EdgeIter nextEdge = currentEdge;
      nextEdge++;

      if (currentEdge->isNew == true){
        if ((currentEdge->halfedge()->vertex()->isNew)^(currentEdge->halfedge()->twin()->vertex()->isNew)){ //XOR isNew for each vertex;
            mesh.flipEdge(currentEdge);
        }
      }

      currentEdge = nextEdge;
    }

    // TODO Finally, copy the new vertex positions into final Vertex::position.
    cout<<"Copying all new vertex positions into old vertices" << endl;
    currentVert = mesh.verticesBegin();
    while(currentVert != mesh.verticesEnd()){
      VertexIter nextVert = currentVert;
      nextVert++;

      currentVert->position = currentVert->newPosition;

      currentVert = nextVert;
    }

    return;
  }


//Sums all neighbouring vertices and sets centroid to sum.
  void Vertex::computeCentroid(){
    HalfedgeCIter start = halfedge();
    start = start->twin();
    HalfedgeCIter current = start;
    Vector3D sum;
    Size counter = 0;

    do{
      sum += current->vertex()->position;
      current = current->next()->twin();
      counter++;
    }while(current != start);

    if (counter != degree()){
      cerr << "Neighbours found not equal to degree" << endl;
      return;
    }

    centroid = sum;
  }
}
