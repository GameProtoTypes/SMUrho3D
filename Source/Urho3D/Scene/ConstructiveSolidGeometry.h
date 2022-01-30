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

#pragma once




class csgjs_csgnode;
typedef csgjs_csgnode* csg_function(const csgjs_csgnode*, const csgjs_csgnode*);

namespace Urho3D
{

class Model;
class Node;
class Geometry;

//% Constructive solid geometry manipulator.
class URHO3D_API CSGManipulator : public Object
{
    URHO3D_OBJECT(CSGManipulator, Object);
public:
    /// Construct.
    explicit CSGManipulator(Context* context);
    virtual ~CSGManipulator();

    static void RegisterObject(Context* context);


    void SetResultNode(Node* node);

    /// Combines geometry of base node with geometry of other node.
    void Union(Node* other);
    /// Subtracts from base node any non-intersecting geometry of other node.
    void Intersection(Node* other);
    /// Subtracts geometry of other node from the base node.
    void Subtract(Node* other);
    /// Bakes result as a single geometry and sets it to base node.
    /// \returns base node that was passed to the constructor.
    Node* BakeSingle();
    /// Bakes result creating separate nodes for disjoint geometry pieces.
    /// \returns pointers of new nodes. One of them is base node passed to the constructor.
    ea::vector<Node*> BakeSeparate();

protected:
    ///
    void PerformAction(Node* other, csg_function action);
    ///
    ea::vector<Geometry*> BakeGeometries(bool disjoint);
    ///
    Model* CreateModelResource(ea::vector<Geometry*> geometries);

    /// Base node that is to be manipulated.
    WeakPtr<Node> baseNode_;
    /// A representation of geometry of node we are manipulating.
    csgjs_csgnode* nodeA_ = nullptr;
};

}
