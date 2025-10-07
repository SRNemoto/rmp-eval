// Copyright (c) 2025 Robotic Systems Integration, Inc.
// Licensed under the MIT License. See LICENSE file in the project root for details.

#include <array>
#include <cmath>
#include <bit>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <utility>
#include <vector>

#include "reporter.h"

namespace Evaluator
{
  struct CountNewLine {};
  class NewLineCounterStream : public std::ostream
  {
    int newLineCount = 0;
  public:
    explicit NewLineCounterStream(std::ostream& argStream)
      : std::ostream(argStream.rdbuf())
    {}

    friend NewLineCounterStream& operator<<(NewLineCounterStream&, CountNewLine);

    // Constrained forwarding template: for any T that is streamable to std::ostream
    template <class T>
    friend auto operator<<(NewLineCounterStream& os, T&& x)
        -> std::enable_if_t<
            std::is_same_v<
              decltype(std::declval<std::ostream&>() << std::forward<T>(x)),
              std::ostream&
            >,
            NewLineCounterStream&
          >;

    // Manipulators (std::endl, std::flush, etc.)
    friend NewLineCounterStream&
    operator<<(NewLineCounterStream& os, std::ostream& (*pf)(std::ostream&));

    friend NewLineCounterStream&
    operator<<(NewLineCounterStream& os, std::ios_base& (*pf)(std::ios_base&));

    int GetNewLineCount() const { return newLineCount; }
  };

  inline NewLineCounterStream&
  operator<<(NewLineCounterStream& os, CountNewLine) {
      os.newLineCount++;
      os.put('\n');
      return os;
  }

  template <class T>
  inline auto operator<<(NewLineCounterStream& os, T&& x)
    -> std::enable_if_t<
        std::is_same_v<
          decltype(std::declval<std::ostream&>() << std::forward<T>(x)),
          std::ostream&
        >,
        NewLineCounterStream&
      >
  {
      static_cast<std::ostream&>(os) << std::forward<T>(x);
      return os; // keep chaining as NewLineCounterStream&
  }

  inline NewLineCounterStream&
  operator<<(NewLineCounterStream& os, std::ostream& (*pf)(std::ostream&)) {
      pf(static_cast<std::ostream&>(os));
      return os;
  }

  inline NewLineCounterStream&
  operator<<(NewLineCounterStream& os, std::ios_base& (*pf)(std::ios_base&)) {
      pf(os);
      return os;
  }

  TableColumn::TableColumn(const std::string_view argLabel, int argWidth,
    ValueGetterType argValueGetter, ValueFormatterType argValueFormatter,
    const std::string_view argCategory)
    : Label(argLabel)
    , Category(argCategory)
    , Width(argWidth)
    , ValueGetter(argValueGetter)
    , ValueFormatter(argValueFormatter)
  {}

  void TableColumn::PrintLabel(std::ostream& stream) const
  {
    stream << std::setw(Width) << Label;
  }

  void TableColumn::DefaultFormatter(std::ostream& stream, uint64_t value, int width)
  {
    stream << std::setw(width) << std::setprecision(PrintPrecision) << value;
  }

  void TableColumn::PrintValue(ReportData& data, std::ostream& stream) const
  {
    ValueFormatter(stream, ValueGetter(data), Width);
  }

  static size_t GetBucketIndex(uint64_t element, uint64_t bucketWidth, size_t bucketCount)
  {
    size_t deviations = element / bucketWidth;
    size_t bucketIndex = std::bit_width(deviations);
    return std::min(bucketIndex, bucketCount - 1);
  }

  static constexpr double NanoToMicro = 0.001;

  TableMaker TableMaker::CreateTableMaker(uint64_t bucketWidth, bool isVerbose)
  {
    TableMaker tableMaker;
    tableMaker.bucketWidth = bucketWidth;
    tableMaker.AddColumn(TableColumn{ "Count", TableColumn::DefaultColumnWidth, [](ReportData& data){ return data.observations; } });
    if (isVerbose)
    {
      tableMaker.AddColumn(TableColumn{ "Min", TableColumn::DefaultColumnWidth,
        [](ReportData& data)
        {
          return static_cast<uint64_t>(data.min * NanoToMicro);
        }
      });
      tableMaker.AddColumn(TableColumn{ "Mean", TableColumn::DefaultColumnWidth,
        [](ReportData& data)
        {
          if (data.observations > 0)
          {
            return static_cast<uint64_t>((data.sum * NanoToMicro) / static_cast<double>(data.observations));
          }
          else
          {
            return static_cast<uint64_t>(0);
          }
        }
      });
      tableMaker.AddColumn(TableColumn{ "Median", TableColumn::DefaultColumnWidth,
        [](ReportData& data)
        {
          return static_cast<uint64_t>(data.median * NanoToMicro);
        }});
    } // end if (isVerbose)

    // Add buckets
    static constexpr auto lastBucket = BucketCount - 1UL;
    static constexpr size_t bufferSize = 32;
    for (size_t index = 0; index < lastBucket; ++index)
    {
      double label = std::round(bucketWidth * std::pow(2, index)) * NanoToMicro;
      char buffer[bufferSize] = {};
      std::snprintf(buffer, bufferSize, "< %.0fus", label);
      tableMaker.AddColumn(TableColumn{ buffer, TableColumn::DefaultColumnWidth,
        [index](ReportData& data) { return data.buckets[index]; },
        [index](std::ostream& stream, uint64_t value, int width)
        {
          if (value == 0)
          {
            TableColumn::DefaultFormatter(stream, value, width);
          }
          else
          {
            stream << BucketColorScheme::GetColor(index);
            TableColumn::DefaultFormatter(stream, value, width);
            stream << BucketColorScheme::GetResetColor();
          }
        },
        BucketColorScheme::GetCategory(index)
      });
    }
    // Add a special label for the last bucket
    char lastBuffer[bufferSize] = {};
    double lastLabel = (bucketWidth * std::pow(2, lastBucket - 1)) * NanoToMicro;
    std::snprintf(lastBuffer, bufferSize, ">= %.0fus", lastLabel);
    tableMaker.AddColumn(TableColumn{ lastBuffer, TableColumn::DefaultColumnWidth,
      [](ReportData& data) { return data.buckets[lastBucket]; },
      [](std::ostream& stream, uint64_t value, int width)
      {
        if (value == 0)
        {
          TableColumn::DefaultFormatter(stream, value, width);
        }
        else
        {
          stream << BucketColorScheme::GetColor(lastBucket);
          TableColumn::DefaultFormatter(stream, value, width);
          stream << BucketColorScheme::GetResetColor();
        }
      },
      BucketColorScheme::GetCategory(lastBucket)
    });

    tableMaker.AddColumn(TableColumn{ "us", TableColumn::DefaultColumnWidth,
      [](ReportData& data)
      {
        return static_cast<uint64_t>((data.max - data.target) * NanoToMicro);
      },
      [bucketWidth](std::ostream& stream, uint64_t value, int width)
      {
        auto bucketIndex = GetBucketIndex(value, bucketWidth * NanoToMicro, BucketCount);
        stream << BucketColorScheme::GetColor(bucketIndex);
        TableColumn::DefaultFormatter(stream, value, width);
        stream << BucketColorScheme::GetResetColor();
      },
      "Max Latency"
    });
    tableMaker.AddColumn(TableColumn{ "index", TableColumn::DefaultColumnWidth,
      [](ReportData& data)
      {
        return static_cast<uint64_t>(data.maxIndex);
      },
      TableColumn::DefaultFormatter,
      "Max Latency"
    });
    return tableMaker;
  }

  TableMaker::TableMaker()
    : columns()
    , rowLabelWidth(DefaultRowLabelWidth)
  {
    columns.reserve(10);
  }

  void TableMaker::AddColumn(TableColumn&& column)
  {
    columns.push_back(std::move(column));
  }

  void TableMaker::OptimizeRowLabelWidth(const std::vector<std::pair<std::string_view, ReportData*>>& reports)
  {
    int maxWidth = DefaultRowLabelWidth;
    for (const auto& [label, _] : reports)
    {
      maxWidth = std::max(maxWidth, static_cast<int>(label.length()));
    }
    rowLabelWidth = maxWidth;
  }

  int TableMaker::GetDigitCount(uint64_t value)
  {
    if (value == 0) return 1;
    int count = 0;
    while (value > 0)
    {
      value /= 10;
      count++;
    }
    return count;
  }

  void TableMaker::OptimizeColumnWidths()
  {
    static constexpr int MinimumColumnWidth = 4;

    // Calculate basic widths based on labels
    for (auto& col : columns)
    {
      int labelWidth = static_cast<int>(col.Label.length());
      col.Width = std::max(labelWidth, MinimumColumnWidth);
    }

    OptimizeColumnWidthsForCategories();
  }

  void TableMaker::OptimizeColumnWidthsFromData(const std::vector<std::pair<std::string_view, ReportData*>>& reports)
  {
    static constexpr int MinimumColumnWidth = 4;

    // Find the maximum value for each column across all reports
    for (auto& col : columns)
    {
      int maxValueWidth = 0;
      for (const auto& [_, dataPtr] : reports)
      {
        if (dataPtr != nullptr)
        {
          uint64_t value = col.ValueGetter(*dataPtr);
          int valueWidth = GetDigitCount(value);
          maxValueWidth = std::max(maxValueWidth, valueWidth);
        }
      }

      int labelWidth = static_cast<int>(col.Label.length());
      col.Width = std::max({maxValueWidth, labelWidth, MinimumColumnWidth});
    }

    OptimizeColumnWidthsForCategories();
  }

  void TableMaker::OptimizeColumnWidthsForCategories()
  {
    static constexpr int SeparatorWidth = 3; // " | "

    // Adjust widths to accommodate category headers
    for (size_t i = 0; i < columns.size(); )
    {
      const auto& col = columns[i];
      if (!col.Category.empty())
      {
        // Find all columns in this category span
        std::string_view category = col.Category;
        size_t spanStart = i;
        size_t spanCount = 0;
        int totalWidth = 0;

        for (size_t j = i; j < columns.size() && columns[j].Category == category; ++j)
        {
          if (spanCount > 0)
            totalWidth += SeparatorWidth;
          totalWidth += columns[j].Width;
          spanCount++;
        }

        // Check if category name fits
        int categoryLength = static_cast<int>(category.length());
        if (spanCount > 0 && totalWidth < categoryLength)
        {
          // Need to expand columns to fit category
          int deficit = categoryLength - totalWidth;
          int perColumn = deficit / static_cast<int>(spanCount);
          int remainder = deficit % static_cast<int>(spanCount);

          // Distribute extra width across columns
          for (size_t j = spanStart; j < spanStart + spanCount; ++j)
          {
            columns[j].Width += perColumn;
            if (remainder > 0)
            {
              columns[j].Width++;
              remainder--;
            }
          }
        }

        i += (spanCount > 0 ? spanCount : 1);
      }
      else
      {
        i++;
      }
    }
  }

  int TableMaker::PrintLabels(std::ostream& stream) const
  {
    static constexpr int SeparatorWidth = 3; // " | "
    static constexpr int PaddingAroundValue = 2; // spaces on each side
    int lineCount = 0;

    // First line: Category headers
    stream << BeginRow;
    stream << std::setfill(' ') << std::setw(rowLabelWidth) << "" << Separator;

    // Process columns and group by category
    for (size_t i = 0; i < columns.size(); )
    {
      const auto& col = columns[i];
      if (col.Category.empty())
      {
        // Non-categorized column - print spaces
        stream << std::setw(col.Width) << "" << Separator;
        i++;
      }
      else
      {
        // Count how many consecutive columns share this category
        std::string_view category = col.Category;
        size_t spanCount = 0;
        int totalWidth = 0;
        for (size_t j = i; j < columns.size() && columns[j].Category == category; ++j)
        {
          if (spanCount > 0)
            totalWidth += SeparatorWidth;
          totalWidth += columns[j].Width;
          spanCount++;
        }

        // Center the category label across the spanned columns
        int padding = (totalWidth - static_cast<int>(category.length())) / 2;
        stream << std::setw(padding + static_cast<int>(category.length())) << category;
        stream << std::setw(totalWidth - padding - static_cast<int>(category.length())) << "" << Separator;

        i += spanCount;
      }
    }
    lineCount = AddNewLine(stream, lineCount);

    // Second line: Column labels
    stream << BeginRow;
    stream << std::setfill(' ') << std::left << std::setw(rowLabelWidth) << "Label" << std::right << Separator;
    for (const auto& col : columns)
    {
      col.PrintLabel(stream);
      stream << Separator;
    }
    lineCount = AddNewLine(stream, lineCount);

    // Separator line
    stream << '|';
    stream << std::setfill(Dash) << std::setw(rowLabelWidth + SeparatorWidth) << DashJoint;
    for (const auto& col : columns)
    {
      stream << std::string(col.Width + PaddingAroundValue, Dash);
      stream << DashJoint;
    }
    lineCount = AddNewLine(stream, lineCount);
    return lineCount;
  }

  int TableMaker::PrintRow(std::string_view rowLabel, ReportData& data, std::ostream& stream) const
  {
    int lineCount = 0;
    stream << BeginRow;
    stream << std::setfill(' ') << std::left << std::setw(rowLabelWidth) << rowLabel << std::right << Separator;
    for (const auto& col : columns)
    {
      col.PrintValue(data, stream);
      stream << Separator;
    }
    lineCount = AddNewLine(stream, lineCount);
    return lineCount;
  }

  int TableMaker::AddNewLine(std::ostream& stream, int count)
  {
    stream << "\n";
    return count + 1;
  }

  void TableMaker::PrintMaxLatencySummary(std::ostream& stream, std::string_view label, const ReportData& data) const
  {
    uint64_t periodMicros = static_cast<uint64_t>(data.max * NanoToMicro);
    uint64_t latencyMicros = static_cast<uint64_t>((data.max - data.target) * NanoToMicro);
    size_t bucketIndex = GetBucketIndex(latencyMicros, bucketWidth * NanoToMicro, BucketCount);

    stream << std::setw(rowLabelWidth) << label << " max period: "
           << BucketColorScheme::GetColor(bucketIndex) << periodMicros << "µs" << BucketColorScheme::GetResetColor()
           << " at index " << data.maxIndex << " which is "
           << BucketColorScheme::GetColor(bucketIndex) << BucketColorScheme::GetCategory(bucketIndex)
           << BucketColorScheme::GetResetColor() << ".\n";
  }

  TimerReport::TimerReport(uint64_t argTarget, uint64_t argBucketWidth, ReportData* argUpload)
    : uploadLocation(argUpload)
    , target(argTarget)
    , bucketWidth(argBucketWidth)
  {}

  // we don't currently protect it with mutexes because this is only used for read-only printing at the moment
  // and we want to keep the real time threads lockless for now.
  // We need to look into using priority-inheriting and robust mutexes if we want to use mutexes.
  // Alternatively, if we get mangled data, we may try atomic operations instead.
  ReportData TimerReport::Snapshot() const
  {
    ReportData data;
    data.min = min;
    data.max = max;
    data.sum = sum;
    data.minIndex = minIndex;
    data.maxIndex = maxIndex;
    data.observations = observations;
    data.median = median.GetQuantile();
    data.target = target;
    data.bucketWidth = bucketWidth;
    std::memcpy(data.buckets, buckets, sizeof(buckets));
    return data;
  }

  void TimerReport::AddObservation(uint64_t observation, int index)
  {
    observations++;
    sum += observation;
    median.AddObservation(observation);

    if (observation < min)
    {
      min = observation;
      minIndex = index;
    }
    if (observation > max)
    {
      max = observation;
      maxIndex = index;
    }
    
    int64_t difference = std::cmp_greater_equal(observation, target) ? (observation - target) : 0;
    ++buckets[GetBucketIndex(difference, bucketWidth, BucketCount)];

    if (uploadLocation != nullptr)
    {
      auto snapshot = Snapshot();
      *uploadLocation = snapshot;
    }
  }

  // void TimerReport::PrintReport(bool isVerbose, std::ostream& stream) const
  // {
  //   PrintReportCountLines(Snapshot(), isVerbose, stream);
  // }

  DurationReporter::DurationReporter(const std::string& msg)
    : msg_(msg)
    , start_(std::chrono::steady_clock::now())
  {}

  int FormatDuration(std::chrono::milliseconds duration, std::ostream& stream)
  {
    stream << "Duration: ";
    static constexpr int64_t MilliPerSec = 1000;
    auto milliseconds = duration.count() % MilliPerSec;
    const int64_t fullSeconds = duration.count() / MilliPerSec;
    static constexpr int64_t SecPerMin = 60;
    const int64_t seconds = fullSeconds % SecPerMin;
    const int64_t minutes = (fullSeconds / SecPerMin) % SecPerMin;
    const int64_t hours = (fullSeconds / (SecPerMin * SecPerMin));
    stream << std::setfill('0') 
      << std::setw(2) << hours << ":"
      << std::setw(2) << minutes << ":"
      << std::setw(2) << seconds << "."
      << std::setw(3) << milliseconds << "\n";
    return 1;
  }

  int FormatDuration(std::chrono::steady_clock::time_point startTime, std::chrono::steady_clock::time_point endTime, std::ostream& stream)
  {
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    return FormatDuration(duration, stream);
  }

  DurationReporter::~DurationReporter() 
  {
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_);
    std::stringstream stream;
    FormatDuration(duration, stream);
    std::cout << msg_ << " Duration: " << stream.str() << std::endl;
  }
} // end namespace Evaluator
