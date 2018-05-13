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
    int n = evaluatedLevels.back().size();
    if (n <= 1) return;
    
    std::vector<Vector2D> newSet = std::vector<Vector2D>(n-1);
    for (int i = 0; i < n - 1; i++)
    {
      Vector2D p1 = evaluatedLevels.back().at(i);
      Vector2D p2 = evaluatedLevels.back().at(i+1);
      newSet[i] = (1-t) * p1 + t * p2;
    }
    evaluatedLevels.push_back(newSet);
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
    std::vector<Vector3D> q = std::vector<Vector3D>(controlPoints.size());
    for (int i = 0; i < controlPoints.size(); i++)
    {
      q[i] = evaluate1D(controlPoints[i], u);
    }
    return evaluate1D(q, v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.

    Vector3D p1 = points[0];
    Vector3D p2 = points[1];
    Vector3D p3 = points[2];
    Vector3D p4 = points[3];

    p1 = (1-t)*p1 + t*p2;
    p2 = (1-t)*p2 + t*p3;
    p3 = (1-t)*p3 + t*p4;
    
    p1 = (1-t)*p1 + t*p2;
    p2 = (1-t)*p2 + t*p3;

    p1 = (1-t)*p1 + t*p2;

    return p1;
  }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.

    // initialize a vector to store your normal sum
    Vector3D n(0,0,0);
    // since we're in a Vertex, this returns a halfedge pointing _away_ from that vertex
    HalfedgeCIter h_orig = halfedge();
    HalfedgeCIter h = h_orig;

    // keep track of triangle edges
    Vector3D lastEdge(0,0,0), curEdge(0,0,0);

    // get first triangle edge
    Vector3D source_p = h->vertex()->position;
    h = h->twin();
    curEdge = h->vertex()->position - source_p;
  
    h = h->next();
    do {
      h = h->twin();
      lastEdge = curEdge;
      curEdge = h->vertex()->position - source_p;
      n += cross(curEdge, lastEdge);
      h = h->next();
    } while (h != h_orig);

    return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
    if (e0->isBoundary() || e0->halfedge()->twin()->edge()->isBoundary())
    {
      return e0;
    }

    // ******* PHASE I: COLLECT ELEMENTS *******
    // HALFEDGES
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
    // VERTICIES
    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();
    // EDGES
          // e0 is already defined
    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();
    // FACES
    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    // ******* PHASE Ib: ALLOCATE NEW ELEMENTS *******
    // pass

    // ******* PHASE II: REASSIGN ELEMENTS *******
    // HALFEDGES
    // void setNeighbors( HalfedgeIter next,
    //                         HalfedgeIter twin,
    //                         VertexIter vertex,
    //                         EdgeIter edge,
    //                         FaceIter face )
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
    // VERTICIES
    v0->halfedge() = h2;
    v1->halfedge() = h5;
    v2->halfedge() = h3;
    v3->halfedge() = h0;
    // EDGES
    e0->halfedge() = h0;
    e1->halfedge() = h5;
    e2->halfedge() = h1;
    e3->halfedge() = h2;
    e4->halfedge() = h4;
    // FACES
    f0->halfedge() = h0;
    f1->halfedge() = h3;

    // ******* PHASE IIb: DELETE UNUSED ALLOWCATED NEW ELELMENTS *******
    // pass

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.

    if (e0->isBoundary() || e0->halfedge()->twin()->edge()->isBoundary())
    { 
      return e0->halfedge()->vertex();

       // ******* EXTRA CREDIT ON BOUNDRY ATTEMPT *******
      // // set e0 to be the opposite edge of boundary edge
      // if (!e0->isBoundary()) {
      //   e0 = e0->halfedge()->twin()->edge();
      // }

      // // ******* PHASE I: COLLECT ELEMENTS *******
      // // HALFEDGES
      // HalfedgeIter h0 = e0->halfedge();
      // HalfedgeIter h1 = h0->next();
      // HalfedgeIter h2 = h1->next();
      // HalfedgeIter h3 = h0->twin();
      // HalfedgeIter h4 = h1->twin();
      // HalfedgeIter h5 = h2->twin();
      // // VERTICIES
      // VertexIter v0 = h0->vertex();
      // VertexIter v1 = h1->vertex();
      // VertexIter v2 = h2->vertex();
      // // EDGES
      //       // e0 is already defined
      // EdgeIter e1 = h1->edge();
      // EdgeIter e2 = h2->edge();
      // // FACES
      // FaceIter f0 = h0->face();

      // // ******* PHASE Ib: ALLOCATE NEW ELEMENTS *******
      // // HALFEDGES
      // HalfedgeIter h6 = newHalfedge();
      // HalfedgeIter h7 = newHalfedge();
      // HalfedgeIter h8 = newHalfedge();
      // HalfedgeIter h9 = newHalfedge();
      // // VERTICIES
      // VertexIter m = newVertex();
      // // assign vertex's new position
      // m->position = (v0->position + v1->position) / 2;
      // // EDGES
      // EdgeIter e3 = newEdge();
      // EdgeIter e4 = newEdge();
      // // FACES
      // FaceIter f2 = newFace();

      // // ******* PHASE II: REASSIGN ELEMENTS *******
      // // HALFEDGES
      // // void setNeighbors( HalfedgeIter next,
      // //                         HalfedgeIter twin,
      // //                         VertexIter vertex,
      // //                         EdgeIter edge,
      // //                         FaceIter face )
      // h0->setNeighbors(h1, h3,  m, e0, f0);
      // h1->setNeighbors(h9, h4, v1, e1, f0);
      // h2->setNeighbors(h7, h5, v2, e2, f1);
      // h3->setNeighbors();
      // h4->setNeighbors();
      // h5->setNeighbors();
      // h6->setNeighbors();
      // h7->setNeighbors();
      // h8->setNeighbors();
      // h9->setNeighbors();

      // // VERTICIES
      // v0->halfedge() = 
      // v1->halfedge() = 
      // v2->halfedge() = 
      // m->halfedge()  = 
      // // EDGES
      // e0->halfedge() = 
      // e1->halfedge() = 
      // e2->halfedge() = 
      // e3->halfedge() = 
      // e4->halfedge() = 
      // // FACES
      // f0->halfedge() = 
      // f1->halfedge() = 
    }

    // ******* PHASE I: COLLECT ELEMENTS *******
    // HALFEDGES
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
    // VERTICIES
    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();
    // EDGES
          // e0 is already defined
    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();
    // FACES
    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    // ******* PHASE Ib: ALLOCATE NEW ELEMENTS *******
    // HALFEDGES
    HalfedgeIter h10 = newHalfedge();
    HalfedgeIter h11 = newHalfedge();
    HalfedgeIter h12 = newHalfedge();
    HalfedgeIter h13 = newHalfedge();
    HalfedgeIter h14 = newHalfedge();
    HalfedgeIter h15 = newHalfedge();
    // VERTICIES
    VertexIter m = newVertex();
    // assign vertex's new position
    m->position = (v0->position + v1->position) / 2;
    // EDGES
    EdgeIter e5 = newEdge();
    EdgeIter e6 = newEdge();
    EdgeIter e7 = newEdge();
    // FACES
    FaceIter f2 = newFace();
    FaceIter f3 = newFace();

    // ******* PHASE II: REASSIGN ELEMENTS *******
    // HALFEDGES
    // void setNeighbors( HalfedgeIter next,
    //                         HalfedgeIter twin,
    //                         VertexIter vertex,
    //                         EdgeIter edge,
    //                         FaceIter face )
    h0->setNeighbors(h1,  h3,  m, e0, f0);
    h1->setNeighbors(h10, h6, v1, e1, f0);
    h2->setNeighbors(h12, h7, v2, e2, f2);
    h3->setNeighbors(h15, h0, v1, e0, f1);
    h4->setNeighbors(h14, h8, v0, e3, f3);
    h5->setNeighbors(h3,  h9, v3, e4, f1);

    h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());
    h7->setNeighbors(h7->next(), h2, v0, e2, h7->face());
    h8->setNeighbors(h8->next(), h4, v3, e3, h8->face());
    h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());

    h10->setNeighbors(h0,  h11, v2, e6, f0);
    h11->setNeighbors(h2,  h10,  m, e6, f2);
    h12->setNeighbors(h11, h13, v0, e5, f2);
    h13->setNeighbors(h4,  h12,  m, e5, f3);
    h14->setNeighbors(h13, h15, v3, e7, f3);
    h15->setNeighbors(h5,  h14,  m, e7, f1);
    // VERTICIES
    v0->halfedge() = h4;
    v1->halfedge() = h1;
    v2->halfedge() = h2;
    v3->halfedge() = h5;
    m->halfedge()  = h0;
    // EDGES
    e0->halfedge() = h0;
    e1->halfedge() = h1;
    e2->halfedge() = h2;
    e3->halfedge() = h4;
    e4->halfedge() = h5;
    e5->halfedge() = h13;
    e6->halfedge() = h11;
    e7->halfedge() = h14;
    // FACES
    f0->halfedge() = h0;
    f1->halfedge() = h3;
    f2->halfedge() = h2;
    f3->halfedge() = h4;

    // ******* PHASE IIb: DELETE UNUSED ALLOWCATED NEW ELELMENTS *******
    // pass

    return m;
  }


  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.

    // iterate over all vertices in the mesh
    for(VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      // mark vertex as belonging to the original mesh
      v->isNew = false;
      // calculate vertex degree and sum of neighboring positions
      Size n = 0;
      HalfedgeIter h = v->halfedge();
      Vector3D neighbor_sum(0,0,0);
      do {
        neighbor_sum += h->twin()->vertex()->position;
        n++;
        h = h->twin()->next();
      } while(h != v->halfedge());
      // calculate new position
      // (1 - n*u) * original_position + u * neighbor_position_sum
      double u = (n == 3) ? 3.0/16.0 : 3.0/(8.0*n);
      v->newPosition = (1-n*u)*v->position + u * neighbor_sum;
    }


    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.

    // iterate over all edges in the mesh
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      // set isNew to false
      //e->isNew = false;
      HalfedgeIter h = e->halfedge();
      // find connecting vertices a, b
      VertexIter a = h->vertex(), b = h->twin()->vertex();
      // find opposite vertices c, b across the two faces connected to AB in the original mesh will be
      VertexIter c = h->twin()->next()->twin()->vertex();
      VertexIter d = h->next()->twin()->vertex();
      // calculate new position
      // 3/8 * (A + B) + 1/8 * (C + D)
      e->newPosition = 3.0/8.0 * (a->position + b->position) + 1.0/8.0 * (c->position + d->position);
    }


    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)

    // iterate over all edges in the mesh
    EdgeIter e = mesh.edgesBegin();
    //EdgeIter endEdge = mesh.edgesEnd();
    std::vector<EdgeIter> oldEdges;
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      oldEdges.push_back(e);
    }
    for (int i = 0; i < oldEdges.size(); i++) {
      e = oldEdges[i];
      // split edge and get new vertex
      VertexIter m = mesh.splitEdge(e);
      // (i) flag the vertex returned by the split operation as a new vertex
      m->isNew = true;
      // (ii) flag each outgoing edge as either being new or part of the original mesh.
      HalfedgeIter h = m->halfedge();
      h->edge()->isNew = false;
      h = h->twin()->next();
      h->edge()->isNew = true;
      h = h->twin()->next();
      h->edge()->isNew = false;
      h = h->twin()->next();
      h->edge()->isNew = true;
      // (iii) copy the newPosition field from the edge being split into the newPosition field of the newly inserted vertex.
      m->newPosition = e->newPosition;
    }

    // TODO Now flip any new edge that connects an old and new vertex.

    // iterate over all edges in the mesh
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      // if an edge is a new edge
      if (e->isNew)
      {
        HalfedgeIter h = e->halfedge();
        // find connecting vertices a, b
        VertexIter a = h->vertex(), b = h->twin()->vertex(); 
        // if the new edge connects an old and new vertex.
        if ((a->isNew && !b->isNew) || (!a->isNew && b->isNew))
        {
          mesh.flipEdge(e);
        }
      }
    }

    // TODO Finally, copy the new vertex positions into final Vertex::position.
    // iterate over all vertices in the mesh
    for(VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->position = v->newPosition;
    }

    // reset isNew flag for every vertex and mesh
    for(VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->isNew = false;
    }
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      e->isNew = false;
    }

  }

}
