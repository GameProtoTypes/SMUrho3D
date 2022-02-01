//
// Copyright (c) 2008-2017 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "../Graphics/StaticModel.h"
#include "../Graphics/VertexBuffer.h"
#include "../Graphics/IndexBuffer.h"
#include "../Graphics/Model.h"
#include "../Scene/Node.h"
#include "../Graphics/Geometry.h"
#include "../IO/Log.h"

#include "ConstructiveSolidGeometry.h"
#include <csgjs/csgjs.hpp>


namespace Urho3D
{

// Returns true if two polygons share at least one vertex.
static bool CSGIsPolygonAdjacent(const csgjs_polygon& a, const csgjs_polygon& b)
{
    for (const auto& v1 : a.vertices)
    {
        for (const auto& v2 : b.vertices)
        {
            if (Abs(length(v1.pos - v2.pos)) <= std::numeric_limits<float>::epsilon())
                return true;
        }
    }
    return false;
}

/// Converts geometry of the node to polygon list required by csgjs. Optionally transform matrix may be applied to
/// geometry so that manipulations take into account node position, rotation and scale.
std::vector<csgjs_polygon> CSGStaticModelToPolygons(StaticModel* staticModel,
    const Matrix3x4& transform = Matrix3x4::IDENTITY)
{
    std::vector<csgjs_polygon> list;

    // Transform will be applied to vertices and normals.
    auto geom = staticModel->GetLodGeometry(0, 0);  // TODO: handle all LODs

    const unsigned char* vertexData;
    const unsigned char* indexData;
    unsigned elementSize, indexSize;
    const  ea::vector<VertexElement>* elements;

    geom->GetRawData(vertexData, elementSize, indexData, indexSize, elements);


    bool hasPosition = VertexBuffer::HasElement(*elements, TYPE_VECTOR3, SEM_POSITION);

    if (vertexData && indexData && hasPosition)
    {
        unsigned vertexStart = geom->GetVertexStart();
        unsigned vertexCount = geom->GetVertexCount();
        unsigned indexStart = geom->GetIndexStart();
        unsigned indexCount = geom->GetIndexCount();

        unsigned positionOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);
        unsigned normalOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_NORMAL);
        unsigned texCoordOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR2, SEM_TEXCOORD);



        for (unsigned curIdx = indexStart; curIdx < indexStart + indexCount; curIdx += 3)
        {
            //get indexes
            unsigned i1, i2, i3;
            if (indexSize == 2) {
                i1 = *reinterpret_cast<const unsigned short*>(indexData + curIdx * indexSize);
                i2 = *reinterpret_cast<const unsigned short*>(indexData + (curIdx + 1) * indexSize);
                i3 = *reinterpret_cast<const unsigned short*>(indexData + (curIdx + 2) * indexSize);
            }
            else if (indexSize == 4)
            {
                i1 = *reinterpret_cast<const unsigned*>(indexData + curIdx * indexSize);
                i2 = *reinterpret_cast<const unsigned*>(indexData + (curIdx + 1) * indexSize);
                i3 = *reinterpret_cast<const unsigned*>(indexData + (curIdx + 2) * indexSize);
            }

            //lookup triangle using indexes.
            Vector3 v1 = *reinterpret_cast<const Vector3*>(vertexData + i1 * elementSize + positionOffset);
            Vector3 v2 = *reinterpret_cast<const Vector3*>(vertexData + i2 * elementSize + positionOffset);
            Vector3 v3 = *reinterpret_cast<const Vector3*>(vertexData + i3 * elementSize + positionOffset);

            //Normal
            Vector3 normal1 = *reinterpret_cast<const Vector3*>(vertexData + i1 * elementSize + normalOffset);
            Vector3 normal2 = *reinterpret_cast<const Vector3*>(vertexData + i2 * elementSize + normalOffset);
            Vector3 normal3 = *reinterpret_cast<const Vector3*>(vertexData + i3 * elementSize + normalOffset);

            //UV
            Vector2 UV1 = *reinterpret_cast<const Vector2*>(vertexData + i1 * elementSize + texCoordOffset);
            Vector2 UV2 = *reinterpret_cast<const Vector2*>(vertexData + i2 * elementSize + texCoordOffset);
            Vector2 UV3 = *reinterpret_cast<const Vector2*>(vertexData + i3 * elementSize + texCoordOffset);



            //Apply Transform to vertex and rotate normals.
            v1 = transform * Vector3(v1);
            v2 = transform * Vector3(v2);
            v3 = transform * Vector3(v3);

            normal1 = transform.RotationMatrix() * Vector3(normal1);
            normal2 = transform.RotationMatrix() * Vector3(normal2);
            normal3 = transform.RotationMatrix() * Vector3(normal3);


            std::vector<csgjs_vertex> triangle(3);

            triangle[0].pos = csgjs_vector(v1.x_, v1.y_, v1.z_);
            triangle[1].pos = csgjs_vector(v2.x_, v2.y_, v2.z_);
            triangle[2].pos = csgjs_vector(v3.x_, v3.y_, v3.z_);

            triangle[0].normal = csgjs_vector(normal1.x_, normal1.y_, normal1.z_);
            triangle[1].normal = csgjs_vector(normal2.x_, normal2.y_, normal2.z_);
            triangle[2].normal = csgjs_vector(normal3.x_, normal3.y_, normal3.z_);

            triangle[0].uv = csgjs_vector(UV1.x_, UV1.y_, 0);
            triangle[1].uv = csgjs_vector(UV2.x_, UV2.y_, 0);
            triangle[2].uv = csgjs_vector(UV3.x_, UV3.y_, 0);


            list.emplace_back(csgjs_polygon(std::move(triangle)));
        }

    }


    return list;
}

/// Converts vertices to triangles.
template<typename T>
inline unsigned char* CSGSetIndices(void* indexData, size_t numVertices, size_t p)
{
    auto indices = reinterpret_cast<T*>(indexData);
    for (unsigned j = 2; j < numVertices; j++)
    {
        indices[0] = (T)p;
        indices[1] = (T)(p + j - 1);
        indices[2] = (T)(p + j);
        indices += 3;
    }
    return (unsigned char*)indices;
}

struct PolygonBucket
{
    unsigned vertexCount_ = 0;
    unsigned indexCount_ = 0;
    ea::vector<csgjs_polygon> polygons_;

    bool Contains(const csgjs_polygon& polygon) const
    {
        for (const auto& container_polygon : polygons_)
        {
            if (CSGIsPolygonAdjacent(container_polygon, polygon))
                return true;
        }
        return false;
    }
};

ea::vector<Geometry*> CSGPolygonsToGeometry(const std::vector<csgjs_polygon>& polygons, Context* context,
    const ea::vector<VertexElement>& elements, bool disjoint = false)
{
    ea::vector<PolygonBucket> buckets;

    // Non-disjoint geometries will reside in single bucket.
    if (!disjoint)
        buckets.resize(1);

    for (const auto& poly : polygons)
    {
        // Count vertices and indices.
        auto vertexCount = poly.vertices.size();
        auto indexCount = (poly.vertices.size() - 2) * 3;

        PolygonBucket* target_bucket = nullptr;
        if (disjoint)
        {
            // Sort all vertex positions of adjacent polygons into separate buckets. Used for splitting objects into
            // separate geometries.
            for (auto& bucket : buckets)
            {
                if (bucket.Contains(poly))
                {
                    target_bucket = &bucket;
                    break;
                }
            }

            if (target_bucket == nullptr)
            {
                buckets.push_back(PolygonBucket());
                target_bucket = &buckets.back();
            }
        }
        else
            target_bucket = &buckets.front();

        target_bucket->vertexCount_ += vertexCount;
        target_bucket->indexCount_ += indexCount;
        target_bucket->polygons_.push_back(poly);
    }

    if (disjoint)
    {
        // We most likely ended up with too many separate buckets due to polygons being not sorted. We iterate through
        // existing buckets and merge them if they share any vertices.
        size_t last_num_buckets = buckets.size() + 1;
        while (last_num_buckets != buckets.size())
        {
            last_num_buckets = buckets.size();

            for (auto it = buckets.begin(); it != buckets.end(); it++)
            {
                for (auto jt = buckets.begin(); jt != buckets.end(); jt++)
                {
                    if (it != jt)
                    {
                        for (const auto& polygon : (*jt).polygons_)
                        {
                            if ((*it).Contains(polygon))
                            {
                                (*it).indexCount_ += (*jt).indexCount_;
                                (*it).vertexCount_ += (*jt).vertexCount_;
                                for (auto poly : (*jt).polygons_)
                                    (*it).polygons_.push_back(poly);
                                buckets.erase(jt);
                                // Terminate outter loops too.
                                it = buckets.end() - 1;
                                jt = buckets.end() - 1;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    ea::vector<Geometry*> result;
    for (const auto& bucket : buckets)
    {
        size_t p = 0;
        SharedPtr<VertexBuffer> vb(new VertexBuffer(context));
        SharedPtr<IndexBuffer> ib(new IndexBuffer(context));

        vb->SetShadowed(true);
        vb->SetSize(bucket.vertexCount_, elements);

        ib->SetShadowed(true);
        ib->SetSize(bucket.indexCount_, bucket.vertexCount_ > std::numeric_limits<uint16_t>::max());

        auto* vertexData = static_cast<unsigned char*>(vb->Lock(0, vb->GetVertexCount()));
        auto* indexData = static_cast<unsigned char*>(ib->Lock(0, ib->GetIndexCount()));
        bool bigIndices = ib->GetIndexSize() > sizeof(uint16_t);

        for (const auto& poly : bucket.polygons_)
        {
            for (const auto& vertex : poly.vertices)
            {
                for (unsigned k = 0; k < elements.size(); k++)
                {
                    const auto& el = elements.at(k);
                    switch (el.semantic_)
                    {
                    case SEM_POSITION:
                    {
                        assert(el.type_ == TYPE_VECTOR3);
                        memcpy(reinterpret_cast<Vector3*>(vertexData + el.offset_), &vertex.pos, sizeof(vertex.pos));
                        break;
                    }
                    case SEM_NORMAL:
                    {
                        assert(el.type_ == TYPE_VECTOR3);
                        memcpy(reinterpret_cast<Vector3*>(vertexData + el.offset_), &vertex.normal, sizeof(vertex.normal));
                        break;
                    }
                    case SEM_TEXCOORD:
                    {
                        assert(el.type_ == TYPE_VECTOR2);
                        memcpy(reinterpret_cast<Vector2*>(vertexData + el.offset_), &vertex.uv, sizeof(Vector2));
                        break;
                    }
                    case SEM_COLOR:
                    {
                        assert(el.type_ == TYPE_UBYTE4_NORM || el.type_ == TYPE_UBYTE4);
                        *reinterpret_cast<unsigned*>(vertexData + el.offset_) = vertex.color;
                        break;
                    }
                    case SEM_BINORMAL:  // TODO: recalculate
                    case SEM_TANGENT:  // TODO: recalculate
                    case SEM_BLENDWEIGHTS:  // TODO: ???
                    case SEM_BLENDINDICES:  // TODO: ???
                    case SEM_OBJECTINDEX:  // TODO: ???
                    default:
                        break;
                    }
                }
                vertexData += vb->GetVertexSize();
            }

            if (bigIndices)
                indexData = CSGSetIndices<uint32_t>(indexData, poly.vertices.size(), p);
            else
                indexData = CSGSetIndices<uint16_t>(indexData, poly.vertices.size(), p);

            p += poly.vertices.size();
        }

        vb->Unlock();
        ib->Unlock();

        Geometry* geom = new Geometry(context);
        geom->SetVertexBuffer(0, vb);
        geom->SetIndexBuffer(ib);
        geom->SetDrawRange(TRIANGLE_LIST, 0, ib->GetIndexCount());
        result.push_back(geom);
    }
    return result;
}

CSGManipulator::CSGManipulator(Context* context) : Object(context)
{
}

CSGManipulator::~CSGManipulator()
{
    delete nodeA_;
    nodeA_ = nullptr;
}

void CSGManipulator::RegisterObject(Context* context)
{
    context->RegisterFactory<CSGManipulator>();
}

void CSGManipulator::SetResultNode(Node* node)
{
    baseNode_ = node;
    StaticModel* staticModel = node->GetComponent<StaticModel>();
    if (staticModel == nullptr)
    {
        URHO3D_LOGERROR("Node must contain StaticModel component.");
        return;
    }
    nodeA_ = new csgjs_csgnode(CSGStaticModelToPolygons(staticModel));
}


void CSGManipulator::PerformAction(Node* other, csg_function action)
{
    assert(nodeA_ != nullptr);

    StaticModel* staticModel = other->GetComponent<StaticModel>();
    if (staticModel == nullptr)
    {
        URHO3D_LOGERROR("Node must contain StaticModel component.");
        return;
    }
    // Transformation relative to base node. Gometry of `other` node is moved in such a way that base node appears to be
    // at origin point (0, 0, 0).
    auto transform = other->GetTransform() * baseNode_->GetTransform().Inverse();
    // Create csgjs node from polygons of other node.
    csgjs_csgnode* nodeB = new csgjs_csgnode(CSGStaticModelToPolygons(staticModel, transform));
    // Perform action.
    csgjs_csgnode* nodeC = action(nodeA_, nodeB);
    // Delete old nodes.
    delete nodeA_;
    delete nodeB;
    // New result is a new base node.
    nodeA_ = nodeC;
}

void CSGManipulator::Union(Node* other)
{
    PerformAction(other, &csg_union);
}

void CSGManipulator::Intersection(Node* other)
{
    PerformAction(other, &csg_intersect);
}

void CSGManipulator::Subtract(Node* other)
{
    PerformAction(other, &csg_subtract);
}

ea::vector<Geometry*> CSGManipulator::BakeGeometries(bool disjoint)
{
    assert(nodeA_ != nullptr);

    // TODO: lods, vertex buffers.

    // Bake geometries
    std::vector<csgjs_polygon> polygons = nodeA_->allPolygons();
    const ea::vector<VertexElement>& elements = baseNode_->GetComponent<StaticModel>()->GetLodGeometry(0, 0)->GetVertexBuffer(0)->GetElements();
    return CSGPolygonsToGeometry(polygons, context_, elements, disjoint);
}

Node* CSGManipulator::BakeSingle()
{
    assert(nodeA_ != nullptr);
    auto geometries = BakeGeometries(false);
    assert(geometries.size() == 1);
    Model* newModel = CreateModelResource(geometries);
    // Replace geometries in the model component. Geometries are now visible in the scene.
    baseNode_->GetComponent<StaticModel>()->SetModel(newModel);
    return baseNode_;
}

ea::vector<Node*> CSGManipulator::BakeSeparate()
{
    assert(nodeA_ != nullptr);
    auto geometries = BakeGeometries(false);
    Model* newModel = CreateModelResource(geometries);
    // Replace geometries in the model component. Geometries are now visible in the scene.
    baseNode_->GetComponent<StaticModel>()->SetModel(newModel);

    // TODO: ...
    ea::vector<Node*> nodes;
    nodes.push_back(baseNode_.Get());
    return nodes;
}

Model* CSGManipulator::CreateModelResource(ea::vector<Geometry*> geometries)
{
    // Create new model resource with new geometries.
    auto newModel = new Model(context_);
    newModel->SetNumGeometries(geometries.size());
    unsigned index = 0;
    for (auto newGeometry : geometries)
        newModel->SetGeometry(index++, 0, newGeometry);
    return newModel;
}

}
