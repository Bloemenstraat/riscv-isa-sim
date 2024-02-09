#include "devices.h"
#include "mmu.h"
#include "sim.h"
#include <stdexcept>
#include <iostream>
#include <boost/crc.hpp>

mmio_device_map_t& mmio_device_map()
{
  static mmio_device_map_t device_map;
  return device_map;
}

void bus_t::add_device(reg_t addr, abstract_device_t* dev)
{
  // Searching devices via lower_bound/upper_bound
  // implicitly relies on the underlying std::map 
  // container to sort the keys and provide ordered
  // iteration over this sort, which it does. (python's
  // SortedDict is a good analogy)
  devices[addr] = dev;
}

bool bus_t::load(reg_t addr, size_t len, uint8_t* bytes)
{
  // Find the device with the base address closest to but
  // less than addr (price-is-right search)
  auto it = devices.upper_bound(addr);
  if (devices.empty() || it == devices.begin()) {
    // Either the bus is empty, or there weren't 
    // any items with a base address <= addr
    return false;
  }
  // Found at least one item with base address <= addr
  // The iterator points to the device after this, so
  // go back by one item.
  it--;
  return it->second->load(addr - it->first, len, bytes);
}

bool bus_t::store(reg_t addr, size_t len, const uint8_t* bytes)
{
  // See comments in bus_t::load
  auto it = devices.upper_bound(addr);
  if (devices.empty() || it == devices.begin()) {
    return false;
  }
  it--;
  return it->second->store(addr - it->first, len, bytes);
}

std::pair<reg_t, abstract_device_t*> bus_t::find_device(reg_t addr)
{
  // See comments in bus_t::load
  auto it = devices.upper_bound(addr);
  if (devices.empty() || it == devices.begin()) {
    return std::make_pair((reg_t)0, (abstract_device_t*)NULL);
  }
  it--;
  return std::make_pair(it->first, it->second);
}

mem_t::mem_t(reg_t size)
  : sz(size)
{
  if (size == 0 || size % PGSIZE != 0)
    throw std::runtime_error("memory size must be a positive multiple of 4 KiB");
}

mem_t::~mem_t()
{
  for (auto& entry : sparse_memory_map)
    free(entry.second);
}

bool mem_t::load_store(reg_t addr, size_t len, uint8_t* bytes, bool store)
{
  if (addr + len < addr || addr + len > sz)
    return false;

  while (len > 0) {
    auto n = std::min(PGSIZE - (addr % PGSIZE), reg_t(len));

    if (store)
      memcpy(this->contents(addr), bytes, n);
    else
      memcpy(bytes, this->contents(addr), n);

    addr += n;
    bytes += n;
    len -= n;
  }

  return true;
}

char* mem_t::contents(reg_t addr) {
  reg_t ppn = addr >> PGSHIFT, pgoff = addr % PGSIZE;
  auto search = sparse_memory_map.find(ppn);
  if (search == sparse_memory_map.end()) {
    auto res = (char*)calloc(PGSIZE, 1);
    if (res == nullptr)
      throw std::bad_alloc();
    sparse_memory_map[ppn] = res;
    return res + pgoff;
  }
  return search->second + pgoff;
}

void mem_t::dump(std::ostream& o) {
  const char empty[PGSIZE] = {0};
  for (reg_t i = 0; i < sz; i += PGSIZE) {
    reg_t ppn = i >> PGSHIFT;
    auto search = sparse_memory_map.find(ppn);
    if (search == sparse_memory_map.end()) {
      o.write(empty, PGSIZE);
    } else {
      o.write(sparse_memory_map[ppn], PGSIZE);
    }
  }
}


//####################################################
//---------- CRC16 class -----------------------
//####################################################


CRC16::CRC16(memif_t& mem, sim_t *s, std::shared_ptr<plic_t> plic) : mem(mem), s(s), plic(plic) { 
    std::cout << "#####  Peripheral mounted successfully #####" << std::endl;         
}
CRC16::~CRC16() {
    std::cout << "#####  Peripheral unmounted successfully #####" << std::endl;
}

// Load the data found in the data register address
void CRC16::loadData(uint8_t *target) {
    for (uint32_t i = 0; i < this->length_register; i++) {
        target[i] = this->s->from_target(mem.read_uint8((addr_t)this->data_register+i));
    }
}

// Convert an integer into bytes and load them in the bytes array to allow the peripheral to return them
template<typename T>
void CRC16::convertToBytes(T value,  size_t len , uint8_t *bytes) {
    for (size_t i = 0; i < len; i++) {
        bytes[i] = (value >> (i * 8)) & 0xFF; // Extract each byte
    }
}


// I use the Boost library to calculate the CRC for the peripheral
// With 0xFFFF as initial value
void CRC16::calculateCRC() {
    boost::crc_basic<16> crc(this->divisor_register, 0xFFFF, 0, false, false);
    uint8_t *message = new uint8_t[this->length_register];

    loadData(message);
            
    crc.process_bytes(message, this->length_register);

    this->result_register = crc.checksum();
    this->plic->set_interrupt_level(2, 1);
}

bool CRC16::load(reg_t addr, size_t len, uint8_t* bytes) {

    switch (MIMO_BASE+addr) {
        case DIVISOR_REGISTER:
            convertToBytes(this->divisor_register, len , bytes);
            break;
        case LENGTH_REGISTER:
            convertToBytes(this->length_register, len , bytes);
            break;
        case DATA_REGISTER:
            convertToBytes(this->data_register, len , bytes);
            break;
        case RESULT_REGISTER:
            convertToBytes(this->result_register, len , bytes);
            break;
        default:
            std::cout << std::hex << MIMO_BASE+addr << ": Register address not valid" << std::endl;
            break;

    }

    return true; 
}

bool CRC16::store(reg_t addr, size_t len, const uint8_t* bytes) {

    switch (MIMO_BASE+addr) {
        case DIVISOR_REGISTER:
            this->divisor_register = *((uint32_t*)bytes); 
            break;
        case LENGTH_REGISTER:
            this->length_register = *((uint32_t*)bytes);  
            break;
        case DATA_REGISTER:                    
            for (int i = 0; i < 8; ++i) {
                this->data_register |= static_cast<uint64_t>(bytes[i]) << (8 * i);
            }
            break;

        case RESULT_REGISTER:
            std::cout << "Result register is read-only" << std::endl;
            break;

        case CALC_REGISTER:
            calculateCRC();
            break;

        default:
            std::cout << std::hex << MIMO_BASE+addr << ": Register address not valid" << std::endl;
            break;

    }

    return true; 
}


//###################################################
//###################################################