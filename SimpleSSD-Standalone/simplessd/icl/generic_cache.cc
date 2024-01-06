/*
 * Copyright (C) 2017 CAMELab
 *
 * This file is part of SimpleSSD.
 *
 * SimpleSSD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SimpleSSD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SimpleSSD.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "icl/generic_cache.hh"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <iostream>
#include <cmath>
#include <iomanip>


#include "util/algorithm.hh"
#include "../../util/stopwatch.hh"
#include "../sim/simulator.hh"

namespace SimpleSSD {

namespace ICL {


//bool read_req = false;
//bool write_req = false;
uint32_t erasureIOCount = 0;
uint32_t testing = 0;
uint64_t EIO_count = 0;


//std::vector

GenericCache::GenericCache(ConfigReader &c, FTL::FTL *f, DRAM::AbstractDRAM *d)
    : AbstractCache(c, f, d),
      superPageSize(f->getInfo()->pageSize),
      parallelIO(f->getInfo()->pageCountToMaxPerf),
      lineCountInSuperPage(f->getInfo()->ioUnitInPage),
      lineCountInMaxIO(parallelIO * lineCountInSuperPage),
      waySize(conf.readUint(CONFIG_ICL, ICL_WAY_SIZE)),
      prefetchIOCount(conf.readUint(CONFIG_ICL, ICL_PREFETCH_COUNT)),
      prefetchIORatio(conf.readFloat(CONFIG_ICL, ICL_PREFETCH_RATIO)),
      useReadCaching(conf.readBoolean(CONFIG_ICL, ICL_USE_READ_CACHE)),
      useWriteCaching(conf.readBoolean(CONFIG_ICL, ICL_USE_WRITE_CACHE)),
      useReadPrefetch(conf.readBoolean(CONFIG_ICL, ICL_USE_READ_PREFETCH)),
      gen(rd()),
      dist(std::uniform_int_distribution<uint32_t>(0, waySize - 1)) {
  uint64_t cacheSize = conf.readUint(CONFIG_ICL, ICL_CACHE_SIZE);

  lineSize = superPageSize / lineCountInSuperPage;

  if (lineSize != superPageSize) {
    bSuperPage = true;
  }

  if (!conf.readBoolean(CONFIG_FTL, FTL::FTL_USE_RANDOM_IO_TWEAK)) {
    lineSize = superPageSize;
    lineCountInSuperPage = 1;
    lineCountInMaxIO = parallelIO;
  }

  if (!useReadCaching && !useWriteCaching) {
    return;
  }

  // Fully-associated?
  if (waySize == 0) {
    setSize = 1;
    waySize = MAX(cacheSize / lineSize, 1);
  }
  else {
    setSize = MAX(cacheSize / lineSize / waySize, 1);
  }

  debugprint(
      LOG_ICL_GENERIC_CACHE,
      "CREATE  | Set size %u | Way size %u | Line size %u | Capacity %" PRIu64,
      setSize, waySize, lineSize, (uint64_t)setSize * waySize * lineSize);
  debugprint(LOG_ICL_GENERIC_CACHE,
             "CREATE  | line count in super page %u | line count in max I/O %u",
             lineCountInSuperPage, lineCountInMaxIO);

  cacheData.resize(setSize);

  for (uint32_t i = 0; i < setSize; i++) {
    cacheData[i] = new Line[waySize]();
  }

  evictData.resize(lineCountInSuperPage);

  for (uint32_t i = 0; i < lineCountInSuperPage; i++) {
    evictData[i] = (Line **)calloc(parallelIO, sizeof(Line *));
  }

  prefetchTrigger = std::numeric_limits<uint64_t>::max();

  evictMode = (EVICT_MODE)conf.readInt(CONFIG_ICL, ICL_EVICT_GRANULARITY);
  prefetchMode =
      (PREFETCH_MODE)conf.readInt(CONFIG_ICL, ICL_PREFETCH_GRANULARITY);

  // Set evict policy functional
  policy = (EVICT_POLICY)conf.readInt(CONFIG_ICL, ICL_EVICT_POLICY);

  switch (policy) {
    case POLICY_RANDOM:
      evictFunction = [this](uint32_t, uint64_t &) -> uint32_t {
        return dist(gen);
      };
      compareFunction = [this](Line *a, Line *b) -> Line * {
        if (a && b) {
          return dist(gen) > waySize / 2 ? a : b;
        }
        else if (a || b) {
          return a ? a : b;
        }
        else {
          return nullptr;
        }
      };

      break;
    case POLICY_FIFO:
      evictFunction = [this](uint32_t setIdx, uint64_t &tick) -> uint32_t {
        uint32_t wayIdx = 0;
        uint64_t min = std::numeric_limits<uint64_t>::max();

        for (uint32_t i = 0; i < waySize; i++) {
          tick += getCacheLatency() * 8;
          // pDRAM->read(MAKE_META_ADDR(setIdx, i, offsetof(Line, insertedAt)),
          // 8, tick);

          if (cacheData[setIdx][i].insertedAt < min) {
            min = cacheData[setIdx][i].insertedAt;
            wayIdx = i;
          }
        }

        return wayIdx;
      };
      compareFunction = [](Line *a, Line *b) -> Line * {
        if (a && b) {
          if (a->insertedAt < b->insertedAt) {
            return a;
          }
          else {
            return b;
          }
        }
        else if (a || b) {
          return a ? a : b;
        }
        else {
          return nullptr;
        }
      };

      break;
    case POLICY_LEAST_RECENTLY_USED:
      evictFunction = [this](uint32_t setIdx, uint64_t &tick) -> uint32_t {
        uint32_t wayIdx = 0;
        uint64_t min = std::numeric_limits<uint64_t>::max();

        for (uint32_t i = 0; i < waySize; i++) {
          tick += getCacheLatency() * 8;
          // pDRAM->read(MAKE_META_ADDR(setIdx, i, offsetof(Line,
          // lastAccessed)), 8, tick);

          if (cacheData[setIdx][i].lastAccessed < min) {
            min = cacheData[setIdx][i].lastAccessed;
            wayIdx = i;
          }
        }

        return wayIdx;
      };
      compareFunction = [](Line *a, Line *b) -> Line * {
        if (a && b) {
          if (a->lastAccessed < b->lastAccessed) {
            return a;
          }
          else {
            return b;
          }
        }
        else if (a || b) {
          return a ? a : b;
        }
        else {
          return nullptr;
        }
      };

      break;
    default:
      panic("Undefined cache evict policy");

      break;
  }

  

  memset(&stat, 0, sizeof(stat));
}

GenericCache::~GenericCache() {
  if (!useReadCaching && !useWriteCaching) {
    return;
  }

  for (uint32_t i = 0; i < setSize; i++) {
    delete[] cacheData[i];
  }

  for (uint32_t i = 0; i < lineCountInSuperPage; i++) {
    free(evictData[i]);
  }
}

uint64_t GenericCache::getCacheLatency() {
  static uint64_t latency = conf.readUint(CONFIG_ICL, ICL_CACHE_LATENCY);
  static uint64_t core = conf.readUint(CONFIG_CPU, CPU::CPU_CORE_ICL);

  return (core == 0) ? 0 : latency / core;
}

uint32_t GenericCache::calcSetIndex(uint64_t lca) {
  return lca % setSize;
}

void GenericCache::calcIOPosition(uint64_t lca, uint32_t &row, uint32_t &col) {
  uint32_t tmp = lca % lineCountInMaxIO;

  row = tmp % lineCountInSuperPage;
  col = tmp / lineCountInSuperPage;
}

uint32_t GenericCache::getEmptyWay(uint32_t setIdx, uint64_t &tick) {
  uint32_t retIdx = waySize;
  uint64_t minInsertedAt = std::numeric_limits<uint64_t>::max();

  for (uint32_t wayIdx = 0; wayIdx < waySize; wayIdx++) {
    Line &line = cacheData[setIdx][wayIdx];

    if (!line.valid) {
      tick += getCacheLatency() * 8;
      // pDRAM->read(MAKE_META_ADDR(setIdx, wayIdx, offsetof(Line, insertedAt)),
      // 8, tick);

      if (minInsertedAt > line.insertedAt) {
        minInsertedAt = line.insertedAt;
        retIdx = wayIdx;
      }
    }
  }

  return retIdx;
}

uint32_t GenericCache::getValidWay(uint64_t lca, uint64_t &tick) {
  uint32_t setIdx = calcSetIndex(lca);
  uint32_t wayIdx;

  for (wayIdx = 0; wayIdx < waySize; wayIdx++) {
    Line &line = cacheData[setIdx][wayIdx];

    tick += getCacheLatency() * 8;
    // pDRAM->read(MAKE_META_ADDR(setIdx, wayIdx, offsetof(Line, tag)), 8,
    // tick);

    if (line.valid && line.tag == lca) {
      break;
    }
  }

  return wayIdx;
}

void GenericCache::checkSequential(Request &req, SequentialDetect &data) {
  if (data.lastRequest.reqID == req.reqID) {
    data.lastRequest.range = req.range;
    data.lastRequest.offset = req.offset;
    data.lastRequest.length = req.length;

    return;
  }

  if (data.lastRequest.range.slpn * lineSize + data.lastRequest.offset +
          data.lastRequest.length ==
      req.range.slpn * lineSize + req.offset) {
    if (!data.enabled) {
      data.hitCounter++;
      data.accessCounter += data.lastRequest.offset + data.lastRequest.length;

      if (data.hitCounter >= prefetchIOCount &&
          (float)data.accessCounter / superPageSize >= prefetchIORatio) {
        data.enabled = true;
      }
    }
  }
  else {
    data.enabled = false;
    data.hitCounter = 0;
    data.accessCounter = 0;
  }

  data.lastRequest = req;
}

void GenericCache::evictCache(uint64_t tick, bool flush) {
  FTL::Request reqInternal(lineCountInSuperPage);
  uint64_t beginAt;
  uint64_t finishedAt = tick;

  debugprint(LOG_ICL_GENERIC_CACHE, "----- | Begin eviction");

  for (uint32_t row = 0; row < lineCountInSuperPage; row++) {
    for (uint32_t col = 0; col < parallelIO; col++) {
      beginAt = tick;

      if (evictData[row][col] == nullptr) {
        continue;
      }

      if (evictData[row][col]->valid && evictData[row][col]->dirty) {
        reqInternal.lpn = evictData[row][col]->tag / lineCountInSuperPage;
        reqInternal.ioFlag.reset();
        reqInternal.ioFlag.set(row);

        pFTL->write(reqInternal, beginAt);
      }

      if (flush) {
        evictData[row][col]->valid = false;
        evictData[row][col]->tag = 0;
      }

      evictData[row][col]->insertedAt = beginAt;
      evictData[row][col]->lastAccessed = beginAt;
      evictData[row][col]->dirty = false;
      evictData[row][col] = nullptr;

      finishedAt = MAX(finishedAt, beginAt);
    }
  }

  debugprint(LOG_ICL_GENERIC_CACHE,
             "----- | End eviction | %" PRIu64 " - %" PRIu64 " (%" PRIu64 ")",
             tick, finishedAt, finishedAt - tick);
}

// True when hit
bool GenericCache::read(Request &req, uint64_t &tick) {
  bool ret = false;
   

  debugprint(LOG_ICL_GENERIC_CACHE,
             "READ  | REQ %7u-%-4u | LCA %" PRIu64 " | SIZE %" PRIu64,
             req.reqID, req.reqSubID, req.range.slpn, req.length);

  if (useReadCaching) {
    uint32_t setIdx = calcSetIndex(req.range.slpn);
    uint32_t wayIdx;
    uint64_t arrived = tick;

    if (useReadPrefetch) {
      checkSequential(req, readDetect);
    } 

    wayIdx = getValidWay(req.range.slpn, tick);

    // Do we have valid data?
    if (wayIdx != waySize) {
      uint64_t tickBackup = tick;

      // Wait cache to be valid
      if (tick < cacheData[setIdx][wayIdx].insertedAt) {
        tick = cacheData[setIdx][wayIdx].insertedAt;
      }

      // Update last accessed time
      cacheData[setIdx][wayIdx].lastAccessed = tick;

      // DRAM access
      pDRAM->read(&cacheData[setIdx][wayIdx], req.length, tick);

      debugprint(LOG_ICL_GENERIC_CACHE,
                 "READ  | Cache hit at (%u, %u) | %" PRIu64 " - %" PRIu64
                 " (%" PRIu64 ")",
                 setIdx, wayIdx, arrived, tick, tick - arrived);

      ret = true;

      // Do we need to prefetch data?
      if (useReadPrefetch && req.range.slpn == prefetchTrigger) {
        debugprint(LOG_ICL_GENERIC_CACHE, "READ  | Prefetch triggered");

        req.range.slpn = lastPrefetched;

        // Backup tick
        arrived = tick;
        tick = tickBackup;

        goto ICL_GENERIC_CACHE_READ;
      }
    }
    // We should read data from NVM
    else {
    ICL_GENERIC_CACHE_READ:
      FTL::Request reqInternal(lineCountInSuperPage, req);
      std::vector<std::pair<uint64_t, uint64_t>> readList;
      uint32_t row, col;  // Variable for I/O position (IOFlag)
      uint64_t dramAt;
      uint64_t beginLCA, endLCA;
      uint64_t beginAt, finishedAt = tick;

      if (readDetect.enabled) {
        // TEMP: Disable DRAM calculation for prevent conflict
        pDRAM->setScheduling(false);

        if (!ret) {
          debugprint(LOG_ICL_GENERIC_CACHE, "READ  | Read ahead triggered");
        }

        beginLCA = req.range.slpn;

        // If super-page is disabled, just read all pages from all planes
        if (prefetchMode == MODE_ALL || !bSuperPage) {
          endLCA = beginLCA + lineCountInMaxIO;
          prefetchTrigger = beginLCA + lineCountInMaxIO / 2;
        }
        else {
          endLCA = beginLCA + lineCountInSuperPage;
          prefetchTrigger = beginLCA + lineCountInSuperPage / 2;
        }

        lastPrefetched = endLCA;
      }
      else {
        beginLCA = req.range.slpn;
        endLCA = beginLCA + 1;
      }

      for (uint64_t lca = beginLCA; lca < endLCA; lca++) {
        beginAt = tick;

        // Check cache
        if (getValidWay(lca, beginAt) != waySize) {
          continue;
        }

        // Find way to write data read from NVM
        setIdx = calcSetIndex(lca);
        wayIdx = getEmptyWay(setIdx, beginAt);

        if (wayIdx == waySize) {
          wayIdx = evictFunction(setIdx, beginAt);

          if (cacheData[setIdx][wayIdx].dirty) {
            // We need to evict data before write
            calcIOPosition(cacheData[setIdx][wayIdx].tag, row, col);
            evictData[row][col] = cacheData[setIdx] + wayIdx;
          }
        }

        cacheData[setIdx][wayIdx].insertedAt = beginAt;
        cacheData[setIdx][wayIdx].lastAccessed = beginAt;
        cacheData[setIdx][wayIdx].valid = true;
        cacheData[setIdx][wayIdx].dirty = false;

        readList.push_back({lca, ((uint64_t)setIdx << 32) | wayIdx});

        finishedAt = MAX(finishedAt, beginAt);
      }

      tick = finishedAt;

      evictCache(tick);

      for (auto &iter : readList) {
        Line *pLine = &cacheData[iter.second >> 32][iter.second & 0xFFFFFFFF];

        // Read data
        reqInternal.lpn = iter.first / lineCountInSuperPage;
        reqInternal.ioFlag.reset();
        reqInternal.ioFlag.set(iter.first % lineCountInSuperPage);

        beginAt = tick;  // Ignore cache metadata access

        // If superPageSizeData is true, read first LPN only
        pFTL->read(reqInternal, beginAt);

        // DRAM delay
        dramAt = pLine->insertedAt;
        pDRAM->write(pLine, lineSize, dramAt);

        // Set cache data
        beginAt = MAX(beginAt, dramAt);

        pLine->insertedAt = beginAt;
        pLine->lastAccessed = beginAt;
        pLine->tag = iter.first;

        if (pLine->tag == req.range.slpn) {
          finishedAt = beginAt;
        }

        debugprint(LOG_ICL_GENERIC_CACHE,
                   "READ  | Cache miss at (%u, %u) | %" PRIu64 " - %" PRIu64
                   " (%" PRIu64 ")",
                   iter.second >> 32, iter.second & 0xFFFFFFFF, tick, beginAt,
                   beginAt - tick);
      }

      tick = finishedAt;

      if (readDetect.enabled) {
        if (ret) {
          // This request was prefetch
          debugprint(LOG_ICL_GENERIC_CACHE, "READ  | Prefetch done");

          // Restore tick
          tick = arrived;
        }
        else {
          debugprint(LOG_ICL_GENERIC_CACHE, "READ  | Read ahead done");
        }

        // TEMP: Restore
        pDRAM->setScheduling(true);
      }
    }

    tick += applyLatency(CPU::ICL__GENERIC_CACHE, CPU::READ);
  }
  else {
    FTL::Request reqInternal(lineCountInSuperPage, req);

    pDRAM->write(nullptr, req.length, tick);

    pFTL->read(reqInternal, tick);
  }

  stat.request[0]++; //total reads

  if (ret) {
    stat.cache[0]++; //reads served from cache
  }

  return ret;
}

// True when cold-miss/hit
bool GenericCache::write(Request &req, uint64_t &tick) {


//   const char* path = "/mnt/e/new_simulator/logs/ins.txt";
//  // std::ofstream file(path, std::ios::out | std::ios::trunc);
//    std::ofstream file(path, std::ios::app);

//     // Check if the file is opened successfully
//     if (file.is_open()) {
//         // Iterate through each Line* in the vector and write it to the file
//       //  for (const auto& linePtr : cacheData) {
//             // // Assuming Line has a method 'printToStream' to output its data
//             // file << "Dirty: " << linePtr->dirty << "\n";
//             //  file << "Inserted at: " << linePtr->insertedAt << "\n";
//             //   file << "Last Accessed at: " << linePtr->lastAccessed << "\n";
//             //    file << "Tag: " << linePtr->tag << "\n";
//             //     file << "Valid: " << linePtr->valid << "\n";
//         //printf("Tag %lu\n",linePtr->tag);

//         uint32_t waynumber=getValidWay(req.range.slpn,tick);
//             if (waynumber != waySize) {
//               //the location exists in the cache
//              // if (tick - linePtr -> insertedAt <= 10) {
//                 //time difference within time window
//                 erasureIOCount+=1;
//                // file << "WayNum: " << waynumber << "\n";
//                 // file << "EIO count: " << erasureIOCount << "\n"; 
//             }
//            // }

          


//        // }
//         testing +=2;
//         // Close the file when you're done
//          file << "TEst: " << testing<< "\n";
//           file << "EIO count: " << erasureIOCount << "\n"; 
//         file.close();
//     } else {
//         // Handle the case where the file could not be opened
//         std::cerr << "Error opening the file." << std::endl;
//     }



  bool ret = false;
  uint64_t flash = tick;
  bool dirty = false;
 // bool write_req=true;

  const char* pathB = "/mnt/e/new_simulator/logs/ins.txt";
   // std::ofstream fileB(pathB, std::ios::out | std::ios::trunc);
    std::ofstream fileB(pathB, std::ios::app);

     

  debugprint(LOG_ICL_GENERIC_CACHE,
             "WRITE | REQ %7u-%-4u | LCA %" PRIu64 " | SIZE %" PRIu64,
             req.reqID, req.reqSubID, req.range.slpn, req.length);

  FTL::Request reqInternal(lineCountInSuperPage, req);

  if (req.length < lineSize) {
    dirty = true;
  }
  else {
    pFTL->write(reqInternal, flash);
  }

  if (useWriteCaching) {
    uint32_t setIdx = calcSetIndex(req.range.slpn);
    uint32_t wayIdx;

    wayIdx = getValidWay(req.range.slpn, tick);

   

    // Can we update old data?
    if (wayIdx != waySize) {
      uint64_t arrived = tick;

      uint64_t past_time = cacheData[setIdx][wayIdx].lastAccessed;

      // Wait cache to be valid
      if (tick < cacheData[setIdx][wayIdx].insertedAt) {
        tick = cacheData[setIdx][wayIdx].insertedAt;
      }

      // TODO: TEMPORAL CODE
      // We should only show DRAM latency when cache become dirty
      if (dirty) {
        // Update last accessed time
        cacheData[setIdx][wayIdx].insertedAt = tick;
        cacheData[setIdx][wayIdx].lastAccessed = tick;
      }
      else {
        cacheData[setIdx][wayIdx].insertedAt = flash;
        cacheData[setIdx][wayIdx].lastAccessed = flash;
      }

      // Update last accessed time
      cacheData[setIdx][wayIdx].dirty = dirty;

      // DRAM access
      pDRAM->write(&cacheData[setIdx][wayIdx], req.length, tick);

      debugprint(LOG_ICL_GENERIC_CACHE,
                 "WRITE | Cache hit at (%u, %u) | %" PRIu64 " - %" PRIu64
                 " (%" PRIu64 ")",
                 setIdx, wayIdx, arrived, tick, tick - arrived);

      ret = true;



  //ERASURE-LOGIC*********************************************************
    
    

   // double lastAt = cacheData[setIdx][wayIdx].lastAccessed;
    //double tick_sec = static_cast<double>(tick) * 1e-12;

    fileB << std::fixed << std::setprecision(5);
    fileB << "\n*********######***************\n";
    fileB << "Last At: " << past_time << std::endl;
    fileB << "Tick: " << tick << std::endl;
    //fileB << "Past Time : " << past_time << std::endl;

    uint64_t time_diff = tick - past_time;
    fileB << "Difference: " << time_diff << std::endl;

    if (time_diff <= pow(10,13)) {
        EIO_count++;
        fileB << "aNSWER: " << EIO_count << std::endl;
        std::cout << "ANSWER: " << EIO_count << std::endl;


    }
    //  file << "EIO: " << erasureIOCount << "\n"; 

    }
    else {
      uint64_t arrived = tick;

      wayIdx = getEmptyWay(setIdx, tick);

      // Do we have place to write data?
      if (wayIdx != waySize) {
        // Wait cache to be valid
        if (tick < cacheData[setIdx][wayIdx].insertedAt) {
          tick = cacheData[setIdx][wayIdx].insertedAt;
        }

        // TODO: TEMPORAL CODE
        // We should only show DRAM latency when cache become dirty
        if (dirty) {
          // Update last accessed time
          cacheData[setIdx][wayIdx].insertedAt = tick;
          cacheData[setIdx][wayIdx].lastAccessed = tick;
        }
        else {
          cacheData[setIdx][wayIdx].insertedAt = flash;
          cacheData[setIdx][wayIdx].lastAccessed = flash;
        }

        // Update last accessed time
        cacheData[setIdx][wayIdx].valid = true;
        cacheData[setIdx][wayIdx].dirty = dirty;
        cacheData[setIdx][wayIdx].tag = req.range.slpn;

        // DRAM access
        pDRAM->write(&cacheData[setIdx][wayIdx], req.length, tick);

        ret = true;
      }
      // We have to flush
      else {
        uint32_t row, col;  // Variable for I/O position (IOFlag)
        uint32_t setToFlush = calcSetIndex(req.range.slpn);

        for (setIdx = 0; setIdx < setSize; setIdx++) {
          for (wayIdx = 0; wayIdx < waySize; wayIdx++) {
            if (cacheData[setIdx][wayIdx].valid) {
              calcIOPosition(cacheData[setIdx][wayIdx].tag, row, col);

              evictData[row][col] = compareFunction(evictData[row][col],
                                                    cacheData[setIdx] + wayIdx);
            }
          }
        }

        if (evictMode == MODE_SUPERPAGE) {
          uint32_t row, col;  // Variable for I/O position (IOFlag)

          for (row = 0; row < lineCountInSuperPage; row++) {
            for (col = 0; col < parallelIO - 1; col++) {
              evictData[row][col + 1] =
                  compareFunction(evictData[row][col], evictData[row][col + 1]);
              evictData[row][col] = nullptr;
            }
          }
        }

        // We must flush setToFlush set
        bool have = false;

        for (row = 0; row < lineCountInSuperPage; row++) {
          for (col = 0; col < parallelIO; col++) {
            if (evictData[row][col] &&
                calcSetIndex(evictData[row][col]->tag) == setToFlush) {
              have = true;
            }
          }
        }

        // We don't have setToFlush
        if (!have) {
          Line *pLineToFlush = nullptr;

          for (wayIdx = 0; wayIdx < waySize; wayIdx++) {
            if (cacheData[setToFlush][wayIdx].valid) {
              pLineToFlush =
                  compareFunction(pLineToFlush, cacheData[setToFlush] + wayIdx);
            }
          }

          if (pLineToFlush) {
            calcIOPosition(pLineToFlush->tag, row, col);

            evictData[row][col] = pLineToFlush;
          }
        }

        tick += getCacheLatency() * setSize * waySize * 8;

        evictCache(tick, true);

        // Update cacheline of current request
        setIdx = setToFlush;
        wayIdx = getEmptyWay(setIdx, tick);

        if (wayIdx == waySize) {
          panic("Cache corrupted!");
        }

        // DRAM latency
        pDRAM->write(&cacheData[setIdx][wayIdx], req.length, tick);

        // Update cache data
        cacheData[setIdx][wayIdx].insertedAt = tick;
        cacheData[setIdx][wayIdx].lastAccessed = tick;
        cacheData[setIdx][wayIdx].valid = true;
        cacheData[setIdx][wayIdx].dirty = true;
        cacheData[setIdx][wayIdx].tag = req.range.slpn;
      }

      debugprint(LOG_ICL_GENERIC_CACHE,
                 "WRITE | Cache miss at (%u, %u) | %" PRIu64 " - %" PRIu64
                 " (%" PRIu64 ")",
                 setIdx, wayIdx, arrived, tick, tick - arrived);
    }

    tick += applyLatency(CPU::ICL__GENERIC_CACHE, CPU::WRITE);
  }
  else {
    if (dirty) {
      pFTL->write(reqInternal, tick);
    }
    else {
      tick = flash;
    }

    // TEMP: Disable DRAM calculation for prevent conflict
    pDRAM->setScheduling(false);

    pDRAM->read(nullptr, req.length, tick);

    pDRAM->setScheduling(true);
  }

  stat.request[1]++; //total writes

 // if (1e12 - tick)

  if (ret) {
    stat.cache[1]++; //writes served from cache
  }
  // uint32_t start = flash;

  // uint32_t duration = tick - flash;
  // uint32_t writes = 0;
  // if (duration <= 1e12) {
  //   writes++;

  // }


  return ret;
}

// True when flushed
void GenericCache::flush(LPNRange &range, uint64_t &tick) {
  if (useReadCaching || useWriteCaching) {
    uint64_t ftlTick = tick;
    uint64_t finishedAt = tick;
    FTL::Request reqInternal(lineCountInSuperPage);

    for (uint32_t setIdx = 0; setIdx < setSize; setIdx++) {
      for (uint32_t wayIdx = 0; wayIdx < waySize; wayIdx++) {
        Line &line = cacheData[setIdx][wayIdx];

        tick += getCacheLatency() * 8;

        if (line.tag >= range.slpn && line.tag < range.slpn + range.nlp) {
          if (line.dirty) {
            reqInternal.lpn = line.tag / lineCountInSuperPage;
            reqInternal.ioFlag.set(line.tag % lineCountInSuperPage);

            ftlTick = tick;
            pFTL->write(reqInternal, ftlTick);
            finishedAt = MAX(finishedAt, ftlTick);
          }

          line.valid = false;
        }
      }
    }

    tick = MAX(tick, finishedAt);
    tick += applyLatency(CPU::ICL__GENERIC_CACHE, CPU::FLUSH);
  }
}

// True when hit
void GenericCache::trim(LPNRange &range, uint64_t &tick) {
  if (useReadCaching || useWriteCaching) {
    uint64_t ftlTick = tick;
    uint64_t finishedAt = tick;
    FTL::Request reqInternal(lineCountInSuperPage);

    for (uint32_t setIdx = 0; setIdx < setSize; setIdx++) {
      for (uint32_t wayIdx = 0; wayIdx < waySize; wayIdx++) {
        Line &line = cacheData[setIdx][wayIdx];

        tick += getCacheLatency() * 8;

        if (line.tag >= range.slpn && line.tag < range.slpn + range.nlp) {
          reqInternal.lpn = line.tag / lineCountInSuperPage;
          reqInternal.ioFlag.set(line.tag % lineCountInSuperPage);

          ftlTick = tick;
          pFTL->trim(reqInternal, ftlTick);
          finishedAt = MAX(finishedAt, ftlTick);

          line.valid = false;
        }
      }
    }

    tick = MAX(tick, finishedAt);
    tick += applyLatency(CPU::ICL__GENERIC_CACHE, CPU::TRIM);
  }
}

void GenericCache::format(LPNRange &range, uint64_t &tick) {
  if (useReadCaching || useWriteCaching) {
    uint64_t lpn;
    uint32_t setIdx;
    uint32_t wayIdx;

    for (uint64_t i = 0; i < range.nlp; i++) {
      lpn = range.slpn + i;
      setIdx = calcSetIndex(lpn);
      wayIdx = getValidWay(lpn, tick);

      if (wayIdx != waySize) {
        // Invalidate
        cacheData[setIdx][wayIdx].valid = false;
      }
    }
  }

  // Convert unit
  range.slpn /= lineCountInSuperPage;
  range.nlp = (range.nlp - 1) / lineCountInSuperPage + 1;

  pFTL->format(range, tick);
  tick += applyLatency(CPU::ICL__GENERIC_CACHE, CPU::FORMAT);
}

void GenericCache::getStatList(std::vector<Stats> &list, std::string prefix) {
  Stats temp;

  temp.name = prefix + "generic_cache.read.request_count";
  temp.desc = "Read request count";
  list.push_back(temp);


  temp.name = prefix + "generic_cache.read.from_cache";
  temp.desc = "Read requests that served from cache";
  list.push_back(temp);

  temp.name = prefix + "generic_cache.write.request_count";
  temp.desc = "Write request count";
  list.push_back(temp);

  temp.name = prefix + "generic_cache.write.to_cache";
  temp.desc = "Write requests that served to cache";
  list.push_back(temp);

  

  
}

void GenericCache::getStatValues(std::vector<double> &values) {
  values.push_back(stat.request[0]);
  values.push_back(stat.cache[0]);
  values.push_back(stat.request[1]);
  values.push_back(stat.cache[1]);
  

}

void GenericCache::resetStatValues() {
  memset(&stat, 0, sizeof(stat));
}

//void Utility::utilFunc(Request &req, string type) {
  //   static std::vector<std::pair<Request, uint64_t>> processedRequests;
  //   //processedRequests.push_back(req,begin_time);
  //  // static Stopwatch stopwatch;
    

  //   static uint64_t currentTime = getTick();
  //   //uint32_t erasureIOCount = 0;
  //   const char* path = "/mnt/e/new_simulator/logs/erase.txt";
  //  std::ofstream file(path, std::ios::out | std::ios::trunc); // Open file in append mode

  //   for (auto& processedReq : processedRequests) {
  //       auto& firstReq = processedReq.first;
  //       double requestTime = processedReq.second;

  //       bool sameLocation = (firstReq.range.slpn + firstReq.offset == req.range.slpn + req.offset);
  //       bool isReadAndWrite = (type == "r" && type == "w");
  //       bool isWriteAndWrite = (type == "w" && type == "w");
  //       bool withinTimeWindow = (currentTime - requestTime <= 10);

  //       bool temp = isReadAndWrite || isWriteAndWrite;

  //       if (sameLocation && temp && withinTimeWindow) {
  //           // This is an erasure I/O
  //           erasureIOCount++;
  //           continue; // Skip to the next iteration without erasing the request
  //       }
  //   }
 
  //   // erase the requests that qualify as erasure I/Os after the loop.
  //   // processedRequests.erase(
  //   //     std::remove_if(processedRequests.begin(), processedRequests.end(), [&](auto& processedReq) {
  //   //         auto& firstReq = processedReq.first;
  //   //         double requestTime = processedReq.second;

  //   //         bool sameLocation = (firstReq.range.slpn + firstReq.offset == req.range.slpn + req.offset);
  //   //         bool isReadAndWrite = (firstReq.type == BIL::BIO_READ && req.type == BIL::BIO_WRITE);
  //   //         bool isWriteAndWrite = (firstReq.type == BIL::BIO_WRITE && req.type == BIL::BIO_WRITE);
  //   //         bool withinTimeWindow = (currentTime - requestTime <= 1);

  //   //         return sameLocation && (isReadAndWrite || isWriteAndWrite) && withinTimeWindow;
  //   //     }),
  //   //     processedRequests.end()
  //   // );

  //   // Write the erasure I/O count to the file
  //   file << erasureIOCount << " :req" << std::endl;

  //   // Close the file
  //   file.close();
//}


void Utility::outputValues(std::vector<Stats> &list, std::vector<double> &values) {
    const char* path = "/mnt/e/new_simulator/logs/result.txt";
    std::ofstream file(path);

    if (file.is_open()) {
        if (list.size() != values.size()) {
            std::cerr << "Error: Sizes of 'list' and 'values' vectors do not match" << std::endl;
            return;
        }

        // Output header
        file << "******Statistics******\n" << std::endl;

        for (size_t i = 0; i < list.size(); ++i) {
            const Stats& stat = list[i];
            double value = values[i];

            if (stat.name.rfind("icl", 0) == 0 && !Utility::hasEnding(stat.name, "cache")) {
                file << stat.desc << " - " <<  value << std::endl;
               
            }
        }
         file << "\n\n********************************\n\n";
        //  if (read_req) {
        //    file << "it is a READ instr" << std::endl;
        //  } 
        // if (write_req) {
        //   file << "WRITE instr it is" << std::endl;
        // }
        file << "EIO Count: " << erasureIOCount << std::endl;
        file.close();
        std::cout << "Data written to result.txt" << std::endl;
    } else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
}




}  // namespace ICL

}  // namespace SimpleSSD
