//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_H

#include "tin_creator.h"
#include "tin_creation_cgal_types.h"
#include "cgal/Projection_traits_3_extended.h"
#include "cgal/squared_distance_3_cost.h"
#include "tin_creation_utils.h"



namespace TinCreation {

class TinCreationSimplificationPointSet : public TinCreationStrategy
{
    // --- Typedefs ---
    typedef CGAL::Projection_traits_xy_3_extended<K>                         ProjTraitsXY;
    typedef PS::Vertex_base_2<ProjTraitsXY>                                  VbXY;
    typedef CGAL::Constrained_triangulation_face_base_2<ProjTraitsXY>        FbXY;
    typedef CGAL::Triangulation_data_structure_2<VbXY, FbXY>                 TDSXY;
    typedef CGAL::Constrained_Delaunay_triangulation_2<ProjTraitsXY,
            TDSXY, CGAL::Exact_predicates_tag>                               CDTXY;
    typedef CGAL::Constrained_triangulation_plus_2<CDTXY>                    CTXY;
    typedef PS::Squared_distance_3_cost                                      PSSqDist3Cost;

public:
    TinCreationSimplificationPointSet(double borderSimplificationMaxDistance,
                                      double borderSimplificationMaxLength,
                                      unsigned int minFeaturePolylineSize)
            : m_minFeaturePolylineSize(minFeaturePolylineSize)
    {
        m_borderSimpMaxDistPerZoom = std::vector<FT>{borderSimplificationMaxDistance};
        m_borderSimpMaxLengthPerZoom = std::vector<FT>{borderSimplificationMaxLength};
        setParamsForZoom(0);
    }

    TinCreationSimplificationPointSet(const std::vector<double>& borderSimplificationMaxDistance,
                                      const std::vector<double>& borderSimplificationMaxLength,
                                      unsigned int minFeaturePolylineSize)
            : m_borderSimpMaxDistPerZoom(borderSimplificationMaxDistance)
            , m_borderSimpMaxLengthPerZoom(borderSimplificationMaxLength)
    {
        setParamsForZoom(0);
    }

    Polyhedron create(const std::vector<Point_3>& dataPts,
                      const bool &constrainEasternVertices,
                      const bool &constrainWesternVertices,
                      const bool &constrainNorthernVertices,
                      const bool &constrainSouthernVertices) ;

    /**
     * While the process of preserving border/feature edges is implemented in this base class, the methods using point
     * set simplification differ on the way they simplify the point set
     * Thus, this is the only method that needs to be implemented in the child classes
     * @param pts Points to simplify
     * @return Simplified point set
     */
    virtual std::vector<Point_3> simplify(const std::vector<Point_3>& pts) = 0;

    void setParamsForZoom(const unsigned int& zoom) {
//        if (m_borderSimpMaxDistPerZoom.size() == 0) {
//            std::cerr << "[WARNING::TinCreationSimplificationPointSet] Input edges count per zoom vector is empty, using 1 (default value)" << std::endl;
//            m_borderSimpMaxDist = 1;
//        }
//        else if (m_borderSimpMaxDistPerZoom.size() == 1) {
//            // This means that only the root tolerance was specified, we will infer the tolerance at the desired zoom level by dividing by two the root number for each level
//            if (zoom == 0)
//                m_borderSimpMaxDist = m_borderSimpMaxDistPerZoom[0];
//            else {
//                m_borderSimpMaxDist = m_borderSimpMaxDistPerZoom[0] / pow(2, zoom);
//            }
//        }
//        else if ( zoom < m_borderSimpMaxDistPerZoom.size()) {
//            // Use the approximation tolerance corresponding to the zoom in the vector
//            m_borderSimpMaxDist = m_borderSimpMaxDistPerZoom[zoom];
//        }
//        else {
//            // Use the approximation tolerance corresponding to the last zoom specified in the vector
//            m_borderSimpMaxDist = m_borderSimpMaxDistPerZoom.back();
//        }

        m_borderSimpMaxDist = standardHandlingOfThresholdPerZoom(m_borderSimpMaxDistPerZoom, zoom);
        std::cout << "m_borderSimpMaxDist = " << m_borderSimpMaxDist << std::endl;

//        if (m_borderSimpMaxLengthPerZoom.size() == 0) {
//            std::cerr << "[WARNING::TinCreationSimplificationPointSet] Input edges count per zoom vector is empty, using 1 (default value)" << std::endl;
//            m_borderSimpMaxLength = 1;
//        }
//        else if (m_borderSimpMaxLengthPerZoom.size() == 1) {
//            // This means that only the root tolerance was specified, we will infer the tolerance at the desired zoom level by dividing by two the root number for each level
//            if (zoom == 0)
//                m_borderSimpMaxLength = m_borderSimpMaxLengthPerZoom[0];
//            else {
//                m_borderSimpMaxLength = m_borderSimpMaxLengthPerZoom[0] / pow(2, zoom);
//            }
//        }
//        else if ( zoom < m_borderSimpMaxLengthPerZoom.size()) {
//            // Use the approximation tolerance corresponding to the zoom in the vector
//            m_borderSimpMaxLength = m_borderSimpMaxLengthPerZoom[zoom];
//        }
//        else {
//            // Use the approximation tolerance corresponding to the last zoom specified in the vector
//            m_borderSimpMaxLength = m_borderSimpMaxLengthPerZoom.back();
//        }

        m_borderSimpMaxLength = standardHandlingOfThresholdPerZoom(m_borderSimpMaxLengthPerZoom, zoom);

        // Set further parameters that are exclusive of each point set simplification strategy using setParamsForZoomConcreteStrategy(zoom); in derived classes
    }

    virtual void setParamsForZoomConcreteStrategy(const unsigned int& zoom) = 0;

    /// Convert points to ECEF assuming that they are on a UVH format, and given the limits of the tile
    // This conversion is used by some point set simplification methods requiring metric coordinates
    std::vector<Point_3> convertUVHToECEF(const std::vector<Point_3>& pts) const ;

    /// Convert points from local UVH to ECEF given the limits of the tile
    // This conversion is used by some point set simplification methods requiring metric coordinates
    std::vector<Point_3> convertECEFToUVH(const std::vector<Point_3>& pts) const ;

private:
    double m_borderSimpMaxDist;
    double m_borderSimpMaxScaledSqDist;
    double m_borderSimpMaxLength; // We do not scale this length, as it is relative to the XY plane
    std::vector<double> m_borderSimpMaxDistPerZoom;
    std::vector<double> m_borderSimpMaxLengthPerZoom;
    unsigned int m_minFeaturePolylineSize ;
    CTXY m_cdt;

    /// Imposes the required constraints to the internal CDT structure. Simplified the border/feature polylines when needed
    void imposeConstraints(Polyhedron& surface,
                           const bool &constrainEasternVertices,
                           const bool &constrainWesternVertices,
                           const bool &constrainNorthernVertices,
                           const bool &constrainSouthernVertices) ;

    // Precondition: poly.normalize_border() has been executed, and is valid
    // NOTE: The <X>BorderVertices variables include the corner points!
//    void extractTileVerticesAtBorders(const Polyhedron& poly,
//                                      PointCloud& easternBorderVertices,
//                                      PointCloud& westernBorderVertices,
//                                      PointCloud& northernBorderVertices,
//                                      PointCloud& southernBorderVertices,
//                                      Point_3& cornerVertex00,
//                                      Point_3& cornerVertex01,
//                                      Point_3& cornerVertex10,
//                                      Point_3& cornerVertex11) const;
    void getAllNonBorderVertices(const Polyhedron& poly, PointCloud& nonBorderPts) const ;

    /// Simplify the north or south border points. Points are projected to the YZ plane and simplified there.
    PointCloud simplifyEastOrWestBorder(const PointCloud& pts) const ;

    /// Simplify the north or south border points. Points are projected to the XZ plane and simplified there.
    PointCloud simplifyNorthOrSouthBorder(const PointCloud& pts) const ;

    /// Simplify a set of polylines based on their elevation, while preserving the topology in the XY plane
    Polylines simplifyPolylines(const Polylines& polylines) const ;
};

} // End namespace TinCreation

#endif //EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_H
