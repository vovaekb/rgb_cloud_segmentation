#ifndef SEGMENTATION_ESTIMATION_H
#define SEGMENTATION_ESTIMATION_H

#include <map>
#include <vector>
#include <algorithm>

class SegmentationEstimation
{
public:
    SegmentationEstimation(const std::string &segmentation_file, const std::string &groundtruth_file)
        : file_segmentation_(segmentation_file),
          file_groundtruth_(groundtruth_file)
    {
    }

    void run()
    {
        PCL_INFO("Segmentation estimator is running ...\n");

        loadCloud(file_segmentation_, segmentation_cloud_);
        loadCloud(file_groundtruth_, groundtruth_cloud_);

        computeOverlap();
        computeCovering();
    }

private:
    typedef uint32_t Label;

    struct Segment
    {
        // Number of vertices
        size_t size;

        // Overlaps with other labels
        std::map<Label, std::vector<size_t>> overlap;

        // Register an overlap with a point
        void overlapsWith(Label l, size_t index)
        {
            size += 1;
            if (overlap.count(l))
                overlap[l].push_back(index);
            else
                overlap[l].resize(1, index);
        }

        // Get number of different labels with which segment overlaps
        size_t getNumberOfOverlaps() const
        {
            return overlap.size();
        }

        std::pair<Label, size_t> getLargestOverlap() const
        {
            Label label;
            size_t max = 0;
            using it_type = std::map<Label, std::vector<size_t>>::const_iterator;

            for (it_type li_pair = overlap.begin(); li_pair != overlap.end(); li_pair++)
            {
                if (li_pair->second.size() > max)
                {
                    label = li_pair->first;
                    max = li_pair->second.size();
                }
            }

            return std::make_pair(label, max);
        }
    };

    static bool
    loadCloud(const std::string &filename, pcl::PointCloud<pcl::PointXYZL> &cloud)
    {
        if (pcl::io::loadPCDFile(filename, cloud) < 0)
            return (false);
        return (true);
    }

    int getOverlapSegmentSize(Label l)
    {
        int segment_size = 0;
        for (size_t i = 0; i < segmentation_cloud_.size(); ++i)
        {
            pcl::PointXYZL p = segmentation_cloud_.at(i);
            Label label = p.label;
            if (label == l)
                segment_size += 1;
        }

        return segment_size;
    }

    void computeOverlap()
    {
        for (size_t i = 0; i < groundtruth_cloud_.size(); ++i)
        {
            pcl::PointXYZL pg = groundtruth_cloud_.at(i);
            pcl::PointXYZL ps = segmentation_cloud_.at(i);

            // Add segmented points
            if (pcl::isFinite(pg) && ps.label != 0)
                overlap_map_[pg.label].overlapsWith(ps.label, i);
        }

        PCL_INFO("Check overlap_map\n");

        using it_type = std::map<Label, Segment>::iterator;

        int segment_idx = 0;
        for (it_type ls_pair = overlap_map_.begin(); ls_pair != overlap_map_.end(); ls_pair++, segment_idx++)
        {
            Segment segment = ls_pair->second;

            PCL_INFO("\nSegment %d\n", static_cast<int>(ls_pair->first));

            PCL_INFO("Segment size: %d\n", static_cast<int>(segment.size));

            std::map<Label, std::vector<size_t>> segment_overlaps = segment.overlap;

            using it_type = std::map<Label, std::vector<size_t>>::iterator;

            for (it_type li_pair = segment_overlaps.begin(); li_pair != segment_overlaps.end(); li_pair++)
            {
                Label label = li_pair->first;
                size_t overlap_size = li_pair->second.size();

                PCL_INFO("Overlap %d: %d points\n", static_cast<int>(label), static_cast<int>(overlap_size));
            }
        }
    }

    void computeCovering()
    {
        PCL_INFO("\nComputer covering ...\n");

        auto covering = 0.0;
        auto points_n = static_cast<float>(groundtruth_cloud_.size());
        using it_type = std::map<Label, Segment>::iterator;

        for (it_type ls_pair = overlap_map_.begin(); ls_pair != overlap_map_.end(); ls_pair++)
        {
            PCL_INFO("Segment %d\n", static_cast<int>(ls_pair->first));

            Segment segment = ls_pair->second;
            auto segment_size = static_cast<int>(segment.size);

            PCL_INFO("Segment size: %d\n", segment_size);

            // TODO: Calculate max overlap
            std::vector<float> overlaps;

            std::map<Label, std::vector<size_t>> segment_overlaps = segment.overlap;

            using it_type = std::map<Label, std::vector<size_t>>::iterator;

            for (it_type li_pair = segment_overlaps.begin(); li_pair != segment_overlaps.end(); li_pair++)
            {
                Label label = li_pair->first;
                size_t overlap_size = li_pair->second.size();
                int overlap_segment_size = getOverlapSegmentSize(label);

                PCL_INFO("Overlap with %d (%d points): %d points\n", static_cast<int>(label), static_cast<int>(overlap_segment_size), static_cast<int>(overlap_size));

                int overlap_denominator = segment_size + overlap_segment_size - overlap_size;

                PCL_INFO("Size of union of segments (%d and %d): %d\n", segment_size, static_cast<int>(overlap_segment_size), overlap_denominator);

                float overlap = static_cast<float>(overlap_size) / overlap_denominator;

                PCL_INFO("Overlap: %.3f\n\n", overlap);
                overlaps.push_back(overlap);
            }

            float max_overlap = *std::max_element(overlaps.begin(), overlaps.end());

            PCL_INFO("Max overlap: %.3f\n", max_overlap);

            float segment_covering = segment_size * max_overlap;

            covering += segment_covering;

            PCL_INFO("Segment %d covering: %.3f\n\n", static_cast<int>(ls_pair->first), segment_covering);
        }

        covering = covering / points_n;

        PCL_INFO("Covering: %.3f\n", covering);
    }

    std::map<Label, Segment> overlap_map_;

    std::string file_segmentation_;
    std::string file_groundtruth_;

    pcl::PointCloud<pcl::PointXYZL> segmentation_cloud_;
    pcl::PointCloud<pcl::PointXYZL> groundtruth_cloud_;
};

#endif // SEGMENTATION_ESTIMATION_H
