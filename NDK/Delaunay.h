#ifndef DELAUNAY_H
#define DELAUNAY_H

// Delaunay.h
// A self-contained, object-oriented Delaunay Triangulation implementation
// Features: Bowyer-Watson insertion, Natural Neighbor Coordinates (Sibson weights), and gradient fitting.
// Optimized and structured for easy integration into rendering engines (like Foundry's Nuke).

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <set>
#include <map>
#include <type_traits> // Brought in to support implicit vector casting templates

namespace Delaunay {

    /**
     * @brief Identifiers for the Natural Neighbor interpolation algorithms.
     */
    enum class NaturalNeighborMethod {
        Linear = 0,
        Quadratic,
        Sibson,
        SibsonSquare,
        Farin
    };

    /**
     * @brief A standard 2D Vector struct containing basic vector math operators.
     */
    struct Vector2D {
        double x, y;
        Vector2D() : x(0), y(0) {}
        Vector2D(double x, double y) : x(x), y(y) {}

        // Implicit conversion FROM other mathematical Vector types (e.g., DD::Image::Vector2) dynamically.
        // Utilizes SFINAE bounds to ignore copies matching its own exact type, preventing compilation conflicts.
        template <typename T, typename std::enable_if<!std::is_same<typename std::decay<T>::type, Vector2D>::value, int>::type = 0>
        Vector2D(const T& v) : x(static_cast<double>(v.x)), y(static_cast<double>(v.y)) {}

        // Implicit conversion TO other application Vector types directly.
        template <typename T, typename std::enable_if<!std::is_same<typename std::decay<T>::type, Vector2D>::value, int>::type = 0>
        operator T() const { return T(static_cast<float>(x), static_cast<float>(y)); }

        Vector2D operator+(const Vector2D& o) const { return Vector2D(x + o.x, y + o.y); }
        Vector2D operator-(const Vector2D& o) const { return Vector2D(x - o.x, y - o.y); }
        Vector2D operator*(double s) const { return Vector2D(x * s, y * s); }
        Vector2D operator/(double s) const { return Vector2D(x / s, y / s); }

        double dot(const Vector2D& o) const { return x * o.x + y * o.y; }
        double lengthSq() const { return x * x + y * y; }
        double length() const { return std::sqrt(lengthSq()); }
        bool operator==(const Vector2D& o) const { return x == o.x && y == o.y; }
    };

    /**
     * @brief Information attached to each vertex (e.g., Color values, computed gradients).
     */
    struct VertexColor {
        std::array<double, 4> color = { 0.0, 0.0, 0.0, 0.0 }; /// Typically RGBA
        std::array<Vector2D, 4> gradient;                     /// Gradient for each channel
        int pid = -1;                                         /// Original point ID
    };

    /**
     * @brief Represents a single point in the Delaunay Triangulation.
     */
    struct Vertex {
        Vector2D pos;
        VertexColor info;
    };

    /**
     * @brief Represents a triangle connecting 3 vertices in the triangulation.
     */
    struct Triangle {
        int v[3];                 /// Indices of the 3 vertices
        Vector2D circumcenter;    /// Center of the circle passing through all 3 vertices
        double circumradiusSq;    /// Squared radius of the circumcircle
        bool isBad = false;       /// Flag used during Bowyer-Watson insertion

        /**
         * @brief Checks if a given vertex index belongs to this triangle.
         * @param vi The index of the vertex to check.
         * @return True if the triangle contains the vertex, False otherwise.
         */
        bool containsVertex(int vi) const {
            return v[0] == vi || v[1] == vi || v[2] == vi;
        }
    };

    /**
     * @brief Represents a topological edge between two vertices.
     */
    struct Edge {
        int v0, v1;
        bool operator==(const Edge& o) const {
            return (v0 == o.v0 && v1 == o.v1) || (v0 == o.v1 && v1 == o.v0);
        }
        bool operator<(const Edge& o) const {
            int min1 = std::min(v0, v1), max1 = std::max(v0, v1);
            int min2 = std::min(o.v0, o.v1), max2 = std::max(o.v0, o.v1);
            if (min1 != min2) return min1 < min2;
            return max1 < max2;
        }
    };

    /**
     * @brief Core class containing the Delaunay mesh and mathematical interpolation functions.
     */
    class Triangulation {
    public:
        std::vector<Vertex> vertices;
        std::vector<Triangle> triangles;
        std::vector<Edge> hullEdges; /// Edges forming the outer convex hull

        /**
         * @brief Defines the influence a specific vertex has on a queried coordinate.
         */
        struct Weight {
            int vertexIdx;
            double weight;
            double distanceSq;
        };

    private:
        int superTriIndices[3];

        /**
         * @brief Computes the circumcenter coordinate of 3 given points.
         * @param A First coordinate
         * @param B Second coordinate
         * @param C Third coordinate
         * @return Vector2D The center of the circumscribed circle.
         */
        Vector2D computeCircumcenter(const Vector2D& A, const Vector2D& B, const Vector2D& C) const {
            double D = 2.0 * (A.x * (B.y - C.y) + B.x * (C.y - A.y) + C.x * (A.y - B.y));
            if (std::abs(D) < 1e-12) {
                // Collinear fallback to prevent division by zero
                return (A + B + C) / 3.0;
            }
            double A_sq = A.lengthSq();
            double B_sq = B.lengthSq();
            double C_sq = C.lengthSq();

            double ux = (A_sq * (B.y - C.y) + B_sq * (C.y - A.y) + C_sq * (A.y - B.y)) / D;
            double uy = (A_sq * (C.x - B.x) + B_sq * (A.x - C.x) + C_sq * (B.x - A.x)) / D;

            return Vector2D(ux, uy);
        }

        /**
         * @brief Calculates and caches the circumcenter and squared radius of a Triangle struct.
         * @param t The triangle to modify.
         */
        void computeCircumcircle(Triangle& t) const {
            t.circumcenter = computeCircumcenter(vertices[t.v[0]].pos, vertices[t.v[1]].pos, vertices[t.v[2]].pos);
            t.circumradiusSq = (vertices[t.v[0]].pos - t.circumcenter).lengthSq();
        }

        /**
         * @brief Tests whether a point falls strictly inside a triangle's circumcircle.
         * @param t The triangle providing the circumcircle.
         * @param p The point to query.
         * @return True if inside or on the boundary, False otherwise.
         */
        bool inCircumcircle(const Triangle& t, const Vector2D& p) const {
            double distSq = (p - t.circumcenter).lengthSq();
            return distSq <= t.circumradiusSq + 1e-9; // Small epsilon to account for floating-point inaccuracies
        }

    public:
        /**
         * @brief Adds a new point to the pool before building the triangulation.
         * @param pos The 2D coordinate of the point.
         * @param info Payload containing the color data.
         */
        void addPoint(const Vector2D& pos, const VertexColor& info) {
            vertices.push_back({ pos, info });
        }

        /**
         * @brief Calculates the Delaunay Triangulation using the Bowyer-Watson algorithm.
         */
        void build() {
            triangles.clear();
            hullEdges.clear();
            if (vertices.empty()) return;

            // 1. Create a "Super-Triangle" that fully encloses all points
            // This guarantees every point we insert will fall inside an existing triangle
            double minX = vertices[0].pos.x, minY = vertices[0].pos.y;
            double maxX = minX, maxY = minY;
            for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
                minX = std::min(minX, vertices[i].pos.x); minY = std::min(minY, vertices[i].pos.y);
                maxX = std::max(maxX, vertices[i].pos.x); maxY = std::max(maxY, vertices[i].pos.y);
            }

            double dx = maxX - minX;
            double dy = maxY - minY;
            double deltaMax = std::max(dx, dy);
            double midX = (minX + maxX) / 2.0;
            double midY = (minY + maxY) / 2.0;

            Vertex v1 = { Vector2D(midX - 20 * deltaMax, midY - 20 * deltaMax), VertexColor() };
            Vertex v2 = { Vector2D(midX, midY + 20 * deltaMax), VertexColor() };
            Vertex v3 = { Vector2D(midX + 20 * deltaMax, midY - 20 * deltaMax), VertexColor() };

            int offset = static_cast<int>(vertices.size());
            vertices.push_back(v1);
            vertices.push_back(v2);
            vertices.push_back(v3);
            superTriIndices[0] = offset;
            superTriIndices[1] = offset + 1;
            superTriIndices[2] = offset + 2;

            Triangle superTri = { {offset, offset + 1, offset + 2}, Vector2D(), 0.0, false };
            computeCircumcircle(superTri);
            triangles.push_back(superTri);

            // 2. Bowyer-Watson Point Insertion
            for (int i = 0; i < offset; ++i) {
                std::vector<Edge> polygon;
                std::vector<int> badTriangles;

                // Find all triangles that are no longer valid (point falls inside their circumcircle)
                for (int j = 0; j < static_cast<int>(triangles.size()); ++j) {
                    if (inCircumcircle(triangles[j], vertices[i].pos)) {
                        triangles[j].isBad = true;
                        badTriangles.push_back(j);
                    }
                }

                // Find the boundary of the polygonal cavity created by removing bad triangles
                for (int tIdx : badTriangles) {
                    const Triangle& t = triangles[tIdx];
                    for (int j = 0; j < 3; ++j) {
                        Edge edge = { t.v[j], t.v[(j + 1) % 3] };
                        bool isShared = false;
                        for (int otherIdx : badTriangles) {
                            if (tIdx == otherIdx) continue;
                            const Triangle& other = triangles[otherIdx];
                            if (other.containsVertex(edge.v0) && other.containsVertex(edge.v1)) {
                                isShared = true;
                                break;
                            }
                        }
                        if (!isShared) polygon.push_back(edge);
                    }
                }

                // Remove the bad triangles
                triangles.erase(std::remove_if(triangles.begin(), triangles.end(),
                    [](const Triangle& t) { return t.isBad; }), triangles.end());

                // Re-triangulate the cavity using the new point
                for (const auto& edge : polygon) {
                    Triangle newTri = { {edge.v0, edge.v1, i}, Vector2D(), 0.0, false };
                    // Ensure Counter-Clockwise (CCW) orientation
                    Vector2D p0 = vertices[edge.v0].pos;
                    Vector2D p1 = vertices[edge.v1].pos;
                    Vector2D p2 = vertices[i].pos;
                    double det = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
                    if (det < 0) std::swap(newTri.v[0], newTri.v[1]);

                    computeCircumcircle(newTri);
                    triangles.push_back(newTri);
                }
            }

            // 3. Cleanup: Remove triangles sharing a vertex with the initial super-triangle
            triangles.erase(std::remove_if(triangles.begin(), triangles.end(), [&](const Triangle& t) {
                for (int j = 0; j < 3; ++j) {
                    if (t.v[j] >= offset) return true;
                }
                return false;
                }), triangles.end());

            vertices.resize(offset);

            // Cache the convex hull edges for future fallback operations
            hullEdges = getConvexHullEdges();
        }

        /**
         * @brief Returns a list of edges that form the outer boundary (convex hull).
         * @return std::vector<Edge> Collection of boundary edges.
         */
        std::vector<Edge> getConvexHullEdges() const {
            std::map<Edge, int> edgeCount;
            for (int tIdx = 0; tIdx < static_cast<int>(triangles.size()); ++tIdx) {
                const auto& t = triangles[tIdx];
                for (int i = 0; i < 3; ++i) {
                    edgeCount[{t.v[i], t.v[(i + 1) % 3]}]++;
                }
            }
            std::vector<Edge> hull;
            for (const auto& pair : edgeCount) {
                if (pair.second == 1) { // Edges belonging to exactly 1 triangle are on the boundary
                    hull.push_back(pair.first);
                }
            }
            return hull;
        }

        /**
         * @brief Evaluates which vertex in the entire mesh is closest to the given point.
         * @param p The point to query.
         * @return int Index of the nearest vertex.
         */
        int nearestVertex(const Vector2D& p) const {
            if (vertices.empty()) return -1;
            int bestIdx = 0;
            double minDistSq = (vertices[0].pos - p).lengthSq();
            for (int i = 1; i < static_cast<int>(vertices.size()); ++i) {
                double distSq = (vertices[i].pos - p).lengthSq();
                if (distSq < minDistSq) {
                    minDistSq = distSq;
                    bestIdx = i;
                }
            }
            return bestIdx;
        }

        /**
         * @brief Finds which triangle strictly encompasses a given coordinate.
         * @param p The point to query.
         * @return int Index of the containing triangle, or -1 if the point is strictly outside the mesh (outside the convex hull).
         */
        int locate(const Vector2D& p) const {
            for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
                Vector2D p0 = vertices[triangles[i].v[0]].pos;
                Vector2D p1 = vertices[triangles[i].v[1]].pos;
                Vector2D p2 = vertices[triangles[i].v[2]].pos;

                // Barycentric evaluation using cross-product determinants
                double d0 = (p1.x - p0.x) * (p.y - p0.y) - (p1.y - p0.y) * (p.x - p0.x);
                double d1 = (p2.x - p1.x) * (p.y - p1.y) - (p2.y - p1.y) * (p.x - p1.x);
                double d2 = (p0.x - p2.x) * (p.y - p2.y) - (p0.y - p2.y) * (p.x - p2.x);

                // Because Bowyer-Watson triangles are strictly CCW, all positive means strictly inside.
                if ((d0 >= -1e-9 && d1 >= -1e-9 && d2 >= -1e-9) ||
                    (d0 <= 1e-9 && d1 <= 1e-9 && d2 <= 1e-9)) return i;
            }
            return -1;
        }

        /**
         * @brief Fetches all vertices that are connected to a target vertex via any triangle edge.
         * @param vIdx Target vertex index.
         * @return std::vector<int> List of neighboring vertex indices.
         */
        std::vector<int> getVertexNeighbors(int vIdx) const {
            std::vector<int> neighbors;
            for (int tIdx = 0; tIdx < static_cast<int>(triangles.size()); ++tIdx) {
                const auto& t = triangles[tIdx];
                if (t.containsVertex(vIdx)) {
                    for (int i = 0; i < 3; i++) {
                        if (t.v[i] != vIdx && std::find(neighbors.begin(), neighbors.end(), t.v[i]) == neighbors.end()) {
                            neighbors.push_back(t.v[i]);
                        }
                    }
                }
            }
            return neighbors;
        }

        // --- MATH: INTERPOLATION UTILITIES ---

        /**
         * @brief Pre-calculates positional color gradients using Least Squares.
         * Required for advanced Natural Neighbor modes (Sibson/Farin).
         * @param fallback_radius Search radius for degenerate cases.
         */
        void computeGradients(double fallback_radius) {
            int num_verts = static_cast<int>(vertices.size());
            if (num_verts == 2) {
                // Special Fallback: If only 2 points exist, gradients flow directly between them.
                Vector2D p1 = vertices[0].pos;
                Vector2D p2 = vertices[1].pos;
                Vector2D dir = p2 - p1;
                double len = dir.length();
                if (len > 0) dir = dir / len;

                for (int chan = 0; chan < 4; ++chan) {
                    double magnitude = (vertices[1].info.color[chan] - vertices[0].info.color[chan]);
                    if (len > 0) magnitude /= len;
                    Vector2D grad = dir * (magnitude / 2.0 * fallback_radius);
                    vertices[0].info.gradient[chan] = grad;
                    vertices[1].info.gradient[chan] = grad;
                }
                return;
            }

            // Normal Case: Gradient Estimation via Least Squares
            for (int i = 0; i < num_verts; ++i) {
                std::vector<int> neighbors = getVertexNeighbors(i);
                Vector2D p_i = vertices[i].pos;

                for (int chan = 0; chan < 4; ++chan) {
                    double val_i = vertices[i].info.color[chan];
                    double m00 = 0, m01 = 0, m11 = 0, v0 = 0, v1 = 0;

                    for (int nIdx : neighbors) {
                        Vector2D dp = vertices[nIdx].pos - p_i;
                        double df = vertices[nIdx].info.color[chan] - val_i;
                        double w = 1.0 / (dp.lengthSq() + 1e-9);

                        m00 += w * dp.x * dp.x;
                        m01 += w * dp.x * dp.y;
                        m11 += w * dp.y * dp.y;
                        v0 += w * df * dp.x;
                        v1 += w * df * dp.y;
                    }

                    double det = m00 * m11 - m01 * m01;
                    if (std::abs(det) > 1e-12) {
                        vertices[i].info.gradient[chan] = Vector2D(
                            (m11 * v0 - m01 * v1) / det,
                            (m00 * v1 - m01 * v0) / det
                        );
                    }
                    else {
                        vertices[i].info.gradient[chan] = Vector2D(0, 0);
                    }
                }
            }
        }

        /**
         * @brief Projects a point perpendicularly onto a finite line segment and clamps it to the segment bounds.
         * @param queryPoint The coordinate to project.
         * @param segmentStart The starting point of the segment.
         * @param segmentEnd The ending point of the segment.
         * @return Vector2D The nearest valid point strictly on the segment.
         */
        Vector2D projectPointOntoSegment(const Vector2D& queryPoint, const Vector2D& segmentStart, const Vector2D& segmentEnd) const {
            Vector2D segmentVector = segmentEnd - segmentStart;
            Vector2D queryVector = queryPoint - segmentStart;
            double dotProduct = segmentVector.dot(queryVector);
            double segmentLengthSquared = segmentVector.lengthSq();
            double t = (segmentLengthSquared > 1e-12) ? (dotProduct / segmentLengthSquared) : 0.0;

            t = std::max<double>(0.0, std::min<double>(1.0, t));
            if (t == 0.0) return segmentStart;
            if (t == 1.0) return segmentEnd;

            Vector2D nearestPoint = segmentStart + segmentVector * t;

            // Push slightly towards the inside to prevent precision errors on borders
            Vector2D pushVector = nearestPoint - queryPoint;
            double pushLen = pushVector.length();
            if (pushLen > 1e-9) pushVector = pushVector / pushLen * 0.005;

            return nearestPoint + pushVector;
        }

        /**
         * @brief Locates the absolute closest physical point resting on the geometric convex hull outline.
         * @param queryPoint The coordinate starting outside the hull.
         * @return Vector2D The clamped coordinate.
         */
        Vector2D getNearestPointOnHull(const Vector2D& queryPoint) const {
            Vector2D nearest_pt = queryPoint;
            double shortest_distance = 1e30;

            for (const Edge& edge : hullEdges) {
                const Vector2D& start = vertices[edge.v0].pos;
                const Vector2D& end = vertices[edge.v1].pos;

                Vector2D edge_point = projectPointOntoSegment(queryPoint, start, end);
                double distance = (queryPoint - edge_point).lengthSq();
                if (distance < shortest_distance) {
                    shortest_distance = distance;
                    nearest_pt = edge_point;
                }
            }
            return nearest_pt;
        }

        /**
         * @brief Calculates Natural Neighbor Coordinates (Sibson weights).
         * Uses strict topological edge traversal to map the overlapping "stolen" Voronoi area dynamically.
         * @param p The query coordinate.
         * @param outWeights Vector filled with the final structural weights.
         * @return True if weights were generated successfully, False otherwise.
         */
        bool getNaturalNeighborWeights(const Vector2D& p, std::vector<Weight>& outWeights) const {
            outWeights.clear();
            if (vertices.empty()) return false;

            // Direct vertex match? Return weight of 1.0 immediately.
            for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
                if ((vertices[i].pos - p).lengthSq() < 1e-12) {
                    outWeights.push_back({ i, 1.0, 0.0 });
                    return true;
                }
            }

            std::vector<int> badTriangles;
            for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
                if (inCircumcircle(triangles[i], p)) badTriangles.push_back(i);
            }

            if (badTriangles.empty()) return false; // Query point is outside convex hull

            // Map edges to their corresponding bad triangles
            std::map<std::pair<int, int>, int> directed_edges;
            for (int tIdx : badTriangles) {
                const Triangle& t = triangles[tIdx];
                for (int j = 0; j < 3; ++j) {
                    directed_edges[{t.v[j], t.v[(j + 1) % 3]}] = tIdx;
                }
            }

            // Find boundary of the bad triangles (the cavity)
            std::map<int, int> next_bound;
            int start_node = -1;

            for (int tIdx : badTriangles) {
                const Triangle& t = triangles[tIdx];
                for (int j = 0; j < 3; ++j) {
                    int u = t.v[j];
                    int v = t.v[(j + 1) % 3];
                    // If the reverse edge isn't in another bad triangle, this is a boundary edge
                    if (directed_edges.find({ v, u }) == directed_edges.end()) {
                        next_bound[u] = v;
                        start_node = u;
                    }
                }
            }

            if (start_node == -1) return false;

            // Trace the boundary to form the continuous CCW cavity loop
            std::vector<int> cavity;
            int curr = start_node;
            do {
                cavity.push_back(curr);
                if (next_bound.find(curr) == next_bound.end()) return false; // Safety: Broken topological loop
                curr = next_bound[curr];
            } while (curr != start_node && cavity.size() <= vertices.size() + 3);

            if (cavity.size() < 3) return false;

            double totalStolenArea = 0.0;

            // For each vertex on the boundary, compute its stolen Voronoi area topologically
            for (size_t i = 0; i < cavity.size(); ++i) {
                int u = cavity[(i == 0) ? cavity.size() - 1 : i - 1];
                int v = cavity[i];
                int w = cavity[(i + 1) % cavity.size()];

                // Get the bad triangles that bound the cavity at vertex v
                int t_in = directed_edges[{u, v}];
                int t_out = directed_edges[{v, w}];

                std::vector<Vector2D> poly_centers;

                // 1. Circumcenter of new triangle (P, v, w)
                poly_centers.push_back(computeCircumcenter(vertices[v].pos, vertices[w].pos, p));

                // 2. Circumcenters of all bad triangles sharing vertex v, ordered CCW
                int curr_tri_idx = t_out;
                int curr_edge_end = w;
                int safety = 0;

                while (curr_tri_idx != t_in) {
                    if (++safety > static_cast<int>(badTriangles.size())) break;

                    poly_centers.push_back(triangles[curr_tri_idx].circumcenter);

                    // Find the third vertex of the current triangle to pivot
                    const Triangle& t = triangles[curr_tri_idx];
                    int third_v = -1;
                    for (int j = 0; j < 3; ++j) {
                        if (t.v[j] != v && t.v[j] != curr_edge_end) {
                            third_v = t.v[j];
                            break;
                        }
                    }

                    if (third_v == -1) break;

                    curr_edge_end = third_v;
                    auto it = directed_edges.find({ v, third_v });
                    if (it == directed_edges.end()) break; // Safety against floating-point edge cases

                    curr_tri_idx = it->second;
                }

                // Push the final bounding bad triangle's circumcenter
                poly_centers.push_back(triangles[t_in].circumcenter);

                // 3. Circumcenter of new triangle (P, u, v)
                poly_centers.push_back(computeCircumcenter(vertices[u].pos, vertices[v].pos, p));

                // Compute area of the resulting Voronoi polygon using the surveyor's formula
                double stolenArea = 0.0;
                Vector2D origin = vertices[v].pos; // Localize to v for floating point precision
                for (size_t k = 0; k < poly_centers.size(); ++k) {
                    Vector2D p1 = poly_centers[k] - origin;
                    Vector2D p2 = poly_centers[(k + 1) % poly_centers.size()] - origin;
                    stolenArea += (p1.x * p2.y - p2.x * p1.y);
                }
                stolenArea = std::abs(stolenArea) * 0.5;

                if (stolenArea > 0) {
                    outWeights.push_back({ v, stolenArea, (vertices[v].pos - p).lengthSq() });
                    totalStolenArea += stolenArea;
                }
            }

            if (totalStolenArea <= 0.0) return false;

            // Normalize weights so they sum to 1.0
            for (auto& w : outWeights) w.weight /= totalStolenArea;
            return true;
        }

        // --- INTERPOLATION API ---

        /**
         * @brief Evaluates the point using standard Nearest Neighbor logic (creates a Voronoi diagram).
         * @param p The queried pixel location.
         * @return std::array<double, 4> The exact RGBA color of the nearest vertex.
         */
        std::array<double, 4> interpolateNearestNeighbor(const Vector2D& p) const {
            if (vertices.empty()) return { 0.0, 0.0, 0.0, 0.0 };
            int nearest = nearestVertex(p);
            return vertices[nearest].info.color;
        }

        /**
         * @brief Evaluates the point using a Triangle Face approximation.
         * Displays the exact mesh boundaries and averages the corners of each Face.
         * @param p The queried pixel location.
         * @return std::array<double, 4> The averaged RGBA color of the containing triangle.
         */
        std::array<double, 4> interpolateNearestTriangle(const Vector2D& p) const {
            if (vertices.empty()) return { 0.0, 0.0, 0.0, 0.0 };

            int face = locate(p);
            if (face == -1) return { 0.0, 0.0, 0.0, 0.0 };

            std::array<double, 4> color = { 0.0, 0.0, 0.0, 0.0 };
            for (int i = 0; i < 3; ++i) {
                int vIdx = triangles[face].v[i];
                for (int chan = 0; chan < 4; ++chan) {
                    color[chan] += vertices[vIdx].info.color[chan];
                }
            }
            for (int chan = 0; chan < 4; ++chan) color[chan] /= 3.0;
            return color;
        }

        /**
         * @brief Evaluates the point using Inverse Distance Weighting logic (IDW).
         * @param p The queried pixel location.
         * @param max_radius Max calculation radius (0 ignores radius logic).
         * @param power Power falloff metric.
         * @return std::array<double, 4> The globally smoothed IDW RGBA array.
         */
        std::array<double, 4> interpolateIDW(const Vector2D& p, double max_radius, double power) const {
            std::array<double, 4> color = { 0.0, 0.0, 0.0, 0.0 };
            if (vertices.empty()) return color;

            double total_weight = 0.0;
            double max_dist_sq = max_radius * max_radius;

            for (const auto& vit : vertices) {
                double dist_sq = (p - vit.pos).lengthSq();
                double weight;

                if (max_radius <= 0.0f) {
                    double dist = std::pow(dist_sq, power / 2.0);
                    weight = 1.0 / (dist + 0.0001);
                }
                else {
                    if (dist_sq > max_dist_sq) continue;
                    weight = std::pow(1.0 - (std::sqrt(dist_sq) / max_radius), power);
                }

                total_weight += weight;
                for (int chan = 0; chan < 4; ++chan) color[chan] += vit.info.color[chan] * weight;
            }

            if (total_weight > 0) {
                for (int chan = 0; chan < 4; ++chan) color[chan] /= total_weight;
            }

            return color;
        }

        /**
         * @brief Advanced Sibson/Natural Neighbor interpolation logic.
         * Includes boundary snapping to ensure clean calculations at the limits of the convex hull.
         * @param p The queried pixel location.
         * @param method Selection of gradient/C1 blending (Linear, Sibson, Farin, etc).
         * @param allow_retry Automatically true, determines if a failed search should snap to the hull and retry.
         * @return std::array<double, 4> Extrapolated and mathematically blended RGBA color array.
         */
        std::array<double, 4> interpolateNaturalNeighbor(Vector2D p, NaturalNeighborMethod method, bool allow_retry = true) const {
            std::array<double, 4> color = { 0.0, 0.0, 0.0, 0.0 };
            if (vertices.empty()) return color;
            if (vertices.size() == 1) return vertices[0].info.color;

            // Handle edge case of exactly 2 vertices gracefully
            // 2 Points is a degenerate case for 2D triangulations. Bypassing gradient math and forcing linear projection.
            if (vertices.size() == 2) {
                Vector2D p1 = vertices[0].pos;
                Vector2D p2 = vertices[1].pos;

                // Project query point to the line segment connecting the two points
                Vector2D projected = projectPointOntoSegment(p, p1, p2);

                double d1 = (projected - p1).length();
                double d2 = (projected - p2).length();
                double total = d1 + d2;

                if (total < 1e-9) return vertices[0].info.color;

                double w1 = d2 / total; // Weight is inversely proportional to distance
                double w2 = d1 / total;

                for (int chan = 0; chan < 4; ++chan) {
                    color[chan] = w1 * vertices[0].info.color[chan] + w2 * vertices[1].info.color[chan];
                }
                return color;
            }

            // For 3+ points, if the point is strictly outside the convex hull, Natural Neighbor weights are undefined.
            // A point is outside the hull if it falls completely outside every valid Delaunay triangle bounds.
            // Snap the query point to the nearest location strictly on the convex hull edge to prevent erratic values.
            if (locate(p) == -1) {
                if (allow_retry) {
                    return interpolateNaturalNeighbor(getNearestPointOnHull(p), method, false);
                }
                return interpolateNearestNeighbor(p); // Fallback failsafe
            }

            std::vector<Weight> weights;
            bool success = getNaturalNeighborWeights(p, weights);
            if (!success) {
                if (allow_retry) return interpolateNaturalNeighbor(getNearestPointOnHull(p), method, false);
                return interpolateNearestNeighbor(p); // Fallback failsafe
            }

            // Optimization: We evaluate all spatial terms (weights, distances) once per vertex,
            // and apply them to all 4 color channels, vastly improving execution speed.
            switch (method) {
            case NaturalNeighborMethod::Linear: {
                for (const auto& w : weights) {
                    for (int chan = 0; chan < 4; ++chan) {
                        color[chan] += w.weight * vertices[w.vertexIdx].info.color[chan];
                    }
                }
                break;
            }
            case NaturalNeighborMethod::Quadratic: { // Spherical
                for (const auto& w : weights) {
                    const auto& v = vertices[w.vertexIdx];
                    for (int chan = 0; chan < 4; ++chan) {
                        color[chan] += w.weight * (v.info.color[chan] + v.info.gradient[chan].dot(p - v.pos) * 0.5);
                    }
                }
                break;
            }
            case NaturalNeighborMethod::Sibson: { // Sibson C1 Blending
                double term1 = 0, term2 = 0, term3 = 0;
                std::array<double, 4> lin_int = { 0,0,0,0 };
                std::array<double, 4> grad_int = { 0,0,0,0 };

                for (const auto& w : weights) {
                    const auto& v = vertices[w.vertexIdx];
                    double distSq = (v.pos - p).lengthSq();
                    double dist = std::sqrt(distSq);

                    // Prevent division by zero if query lands exactly on a point
                    if (distSq < 1e-9) return v.info.color;

                    term1 += w.weight / dist;
                    term2 += w.weight * distSq;
                    term3 += w.weight * dist;

                    for (int chan = 0; chan < 4; ++chan) {
                        lin_int[chan] += w.weight * v.info.color[chan];
                        grad_int[chan] += (w.weight / dist) * (v.info.color[chan] + v.info.gradient[chan].dot(p - v.pos));
                    }
                }
                double term4 = term3 / term1;
                for (int chan = 0; chan < 4; ++chan) {
                    grad_int[chan] /= term1;
                    color[chan] = (term4 * lin_int[chan] + term2 * grad_int[chan]) / (term4 + term2);
                }
                break;
            }
            case NaturalNeighborMethod::SibsonSquare: {
                double term1 = 0, term2 = 0, term3 = 0;
                std::array<double, 4> lin_int = { 0,0,0,0 };
                std::array<double, 4> grad_int = { 0,0,0,0 };

                for (const auto& w : weights) {
                    const auto& v = vertices[w.vertexIdx];
                    double distSq = (v.pos - p).lengthSq();
                    if (distSq < 1e-9) return v.info.color;

                    term1 += w.weight / distSq;
                    term2 += w.weight * distSq;
                    term3 += w.weight;

                    for (int chan = 0; chan < 4; ++chan) {
                        lin_int[chan] += w.weight * v.info.color[chan];
                        grad_int[chan] += (w.weight / distSq) * (v.info.color[chan] + v.info.gradient[chan].dot(p - v.pos));
                    }
                }
                double term4 = term3 / term1;
                for (int chan = 0; chan < 4; ++chan) {
                    grad_int[chan] /= term1;
                    color[chan] = (term4 * lin_int[chan] + term2 * grad_int[chan]) / (term4 + term2);
                }
                break;
            }
            case NaturalNeighborMethod::Farin: { // Farin C1
                int n = static_cast<int>(weights.size());

                // Note: Farin's formulation depends heavily on channel-specific interactions, 
                // so we compute it per-channel to keep the code mathematically clear.
                for (int chan = 0; chan < 4; ++chan) {
                    std::vector<std::vector<double>> ords(n, std::vector<double>(n, 0.0));
                    double result = 0.0;

                    for (int i = 0; i < n; ++i) {
                        const auto& vi = vertices[weights[i].vertexIdx];
                        ords[i][i] = vi.info.color[chan];
                        double coordSq = weights[i].weight * weights[i].weight;
                        result += coordSq * weights[i].weight * ords[i][i];

                        double res_i = 0;
                        for (int j = 0; j < n; ++j) {
                            if (i != j) {
                                const auto& vj = vertices[weights[j].vertexIdx];
                                ords[i][j] = vi.info.gradient[chan].dot(vj.pos - vi.pos);
                                res_i += (3.0 * ords[i][i] + ords[i][j]) * weights[j].weight;
                            }
                        }
                        result += coordSq * res_i;
                    }
                    for (int i = 0; i < n; ++i) {
                        for (int j = i + 1; j < n; ++j) {
                            for (int k = j + 1; k < n; ++k) {
                                result += (2.0 * (ords[i][i] + ords[j][j] + ords[k][k]) +
                                    0.5 * (ords[i][j] + ords[i][k] + ords[j][i] + ords[j][k] + ords[k][i] + ords[k][j])) *
                                    weights[i].weight * weights[j].weight * weights[k].weight;
                            }
                        }
                    }
                    color[chan] = result;
                }
                break;
            }
            }

            return color;
        }
    };
}

#endif // DELAUNAY_H