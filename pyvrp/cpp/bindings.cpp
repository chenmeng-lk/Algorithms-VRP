/*
这个文件是`pyvrp`库的Python绑定实现文件，使用pybind11将C++核心类
（如`ProblemData`、`Solution`、`Route`、`CostEvaluator`等）暴露
给Python接口，使Python代码能够调用C++实现的高性能VRP（车辆路径问题）
求解算法，包括问题数据建模、解决方案构建、成本评估等功能。
*/
#include "bindings.h"
#include "CostEvaluator.h"
#include "DurationSegment.h"
#include "DynamicBitset.h"
#include "LoadSegment.h"
#include "Matrix.h"
#include "ProblemData.h"
#include "RandomNumberGenerator.h"
#include "Route.h"
#include "Solution.h"
#include "Trip.h"
#include "pyvrp_docs.h"

#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <sstream>
#include <string>
#include <variant>

namespace py = pybind11;

using pyvrp::CostEvaluator;
using pyvrp::DurationSegment;
using pyvrp::DynamicBitset;
using pyvrp::LoadSegment;
using pyvrp::Matrix;
using pyvrp::ProblemData;
using pyvrp::RandomNumberGenerator;
using pyvrp::Route;
using pyvrp::Solution;
using pyvrp::Trip;

PYBIND11_MODULE(_pyvrp, m)  // 定义Python模块_pyvrp，这是主要的绑定入口点
{
    py::class_<DynamicBitset>(m, "DynamicBitset", DOC(pyvrp, DynamicBitset))  // 绑定DynamicBitset类到Python
        .def(py::init<size_t>(), py::arg("num_bits"))  // 构造函数，接受一个参数
        .def(py::self == py::self, py::arg("other"))  // this is __eq__ 定义相等操作符
        .def("all", &DynamicBitset::all)  // 绑定all方法
        .def("any", &DynamicBitset::any)  // 绑定any方法
        .def("none", &DynamicBitset::none)  // 绑定none方法
        .def("count", &DynamicBitset::count)  // 绑定count方法
        .def("__len__", &DynamicBitset::size)  // 绑定__len__特殊方法
        .def("set", &DynamicBitset::set)  // 绑定set方法
        .def("reset", &DynamicBitset::reset)  // 绑定reset方法
        .def(
            "__getitem__",
            [](DynamicBitset const &bitset, size_t idx) { return bitset[idx]; },  // 绑定索引访问操作
            py::arg("idx"))
        .def(
            "__setitem__",
            [](DynamicBitset &bitset, size_t idx, bool value)
            { bitset[idx] = value; },  // 绑定索引赋值操作
            py::arg("idx"),
            py::arg("value"))
        .def("__or__", &DynamicBitset::operator|, py::arg("other"))  // 绑定或操作符
        .def("__and__", &DynamicBitset::operator&, py::arg("other"))  // 绑定与操作符
        .def("__xor__", &DynamicBitset::operator^, py::arg("other"))  // 绑定异或操作符
        .def("__invert__", &DynamicBitset::operator~);  // 绑定取反操作符

    py::class_<ProblemData::Client>(
        m, "Client", DOC(pyvrp, ProblemData, Client))  // 绑定ProblemData::Client类
        .def(py::init<pyvrp::Coordinate,
                      pyvrp::Coordinate,
                      std::vector<pyvrp::Load>,
                      std::vector<pyvrp::Load>,
                      pyvrp::Duration,
                      pyvrp::Duration,
                      pyvrp::Duration,
                      pyvrp::Duration,
                      pyvrp::Cost,
                      bool,
                      std::optional<size_t>,
                      char const *>(),
             py::arg("x"),
             py::arg("y"),
             py::arg("delivery") = py::list(),  // 默认参数
             py::arg("pickup") = py::list(),
             py::arg("service_duration") = 0,
             py::arg("tw_early") = 0,
             py::arg("tw_late") = std::numeric_limits<pyvrp::Duration>::max(),
             py::arg("release_time") = 0,
             py::arg("prize") = 0,
             py::arg("required") = true,
             py::arg("group") = py::none(),
             py::kw_only(),  // 后面的参数必须是关键字参数
             py::arg("name") = "")
        .def_readonly("x", &ProblemData::Client::x)  // 暴露只读属性x
        .def_readonly("y", &ProblemData::Client::y)  // 暴露只读属性y
        .def_readonly("delivery",
                      &ProblemData::Client::delivery,
                      py::return_value_policy::reference_internal)  // 返回内部引用
        .def_readonly("pickup",
                      &ProblemData::Client::pickup,
                      py::return_value_policy::reference_internal)
        .def_readonly("service_duration", &ProblemData::Client::serviceDuration)
        .def_readonly("tw_early", &ProblemData::Client::twEarly)
        .def_readonly("tw_late", &ProblemData::Client::twLate)
        .def_readonly("release_time", &ProblemData::Client::releaseTime)
        .def_readonly("prize", &ProblemData::Client::prize)
        .def_readonly("required", &ProblemData::Client::required)
        .def_readonly("group", &ProblemData::Client::group)
        .def_readonly("name",
                      &ProblemData::Client::name,
                      py::return_value_policy::reference_internal)
        .def(py::self == py::self)  // this is __eq__ 定义相等操作符
        .def(py::pickle(
            [](ProblemData::Client const &client) {  // __getstate__ 序列化函数
                return py::make_tuple(client.x,
                                      client.y,
                                      client.delivery,
                                      client.pickup,
                                      client.serviceDuration,
                                      client.twEarly,
                                      client.twLate,
                                      client.releaseTime,
                                      client.prize,
                                      client.required,
                                      client.group,
                                      client.name);
            },
            [](py::tuple t) {  // __setstate__ 反序列化函数
                ProblemData::Client client(
                    t[0].cast<pyvrp::Coordinate>(),         // x
                    t[1].cast<pyvrp::Coordinate>(),         // y
                    t[2].cast<std::vector<pyvrp::Load>>(),  // delivery
                    t[3].cast<std::vector<pyvrp::Load>>(),  // pickup
                    t[4].cast<pyvrp::Duration>(),           // service duration
                    t[5].cast<pyvrp::Duration>(),           // tw early
                    t[6].cast<pyvrp::Duration>(),           // tw late
                    t[7].cast<pyvrp::Duration>(),           // release time
                    t[8].cast<pyvrp::Cost>(),               // prize
                    t[9].cast<bool>(),                      // required
                    t[10].cast<std::optional<size_t>>(),    // group
                    t[11].cast<std::string>());             // name

                return client;
            }))
        .def(
            "__str__",
            [](ProblemData::Client const &client) { return client.name; },
            py::return_value_policy::reference_internal);  // 定义字符串表示

    py::class_<ProblemData::Depot>(m, "Depot", DOC(pyvrp, ProblemData, Depot))  // 绑定ProblemData::Depot类
        .def(py::init<pyvrp::Coordinate,
                      pyvrp::Coordinate,
                      pyvrp::Duration,
                      pyvrp::Duration,
                      char const *>(),
             py::arg("x"),
             py::arg("y"),
             py::arg("tw_early") = 0,
             py::arg("tw_late") = std::numeric_limits<pyvrp::Duration>::max(),
             py::kw_only(),
             py::arg("name") = "")
        .def_readonly("x", &ProblemData::Depot::x)
        .def_readonly("y", &ProblemData::Depot::y)
        .def_readonly("tw_early", &ProblemData::Depot::twEarly)
        .def_readonly("tw_late", &ProblemData::Depot::twLate)
        .def_readonly("name",
                      &ProblemData::Depot::name,
                      py::return_value_policy::reference_internal)
        .def(py::self == py::self)  // this is __eq__
        .def(py::pickle(
            [](ProblemData::Depot const &depot) {  // __getstate__
                return py::make_tuple(
                    depot.x, depot.y, depot.twEarly, depot.twLate, depot.name);
            },
            [](py::tuple t) {  // __setstate__
                ProblemData::Depot depot(
                    t[0].cast<pyvrp::Coordinate>(),  // x
                    t[1].cast<pyvrp::Coordinate>(),  // y
                    t[2].cast<pyvrp::Duration>(),    // tw early
                    t[3].cast<pyvrp::Duration>(),    // tw late
                    t[4].cast<std::string>());       // name

                return depot;
            }))
        .def(
            "__str__",
            [](ProblemData::Depot const &depot) { return depot.name; },
            py::return_value_policy::reference_internal);

    py::class_<ProblemData::ClientGroup>(
        m, "ClientGroup", DOC(pyvrp, ProblemData, ClientGroup))  // 绑定ProblemData::ClientGroup类
        .def(py::init<std::vector<size_t>, bool, char const *>(),
             py::arg("clients") = py::list(),
             py::arg("required") = true,
             py::kw_only(),
             py::arg("name") = "")
        .def("add_client",
             &ProblemData::ClientGroup::addClient,
             py::arg("client"))  // 绑定add_client方法
        .def("clear", &ProblemData::ClientGroup::clear)  // 绑定clear方法
        .def_property_readonly("clients",
                               &ProblemData::ClientGroup::clients,
                               py::return_value_policy::reference_internal)  // 只读属性clients
        .def_readonly("required", &ProblemData::ClientGroup::required)
        .def_readonly("mutually_exclusive",
                      &ProblemData::ClientGroup::mutuallyExclusive)
        .def_readonly("name",
                      &ProblemData::ClientGroup::name,
                      py::return_value_policy::reference_internal)
        .def(py::self == py::self)  // this is __eq__
        .def(py::pickle(
            [](ProblemData::ClientGroup const &group) {  // __getstate__
                return py::make_tuple(
                    group.clients(), group.required, group.name);
            },
            [](py::tuple t) {  // __setstate__
                ProblemData::ClientGroup group(
                    t[0].cast<std::vector<size_t>>(),  // clients
                    t[1].cast<bool>(),                 // required
                    t[2].cast<std::string>());         // name

                return group;
            }))
        .def("__len__", &ProblemData::ClientGroup::size)  // 绑定__len__方法
        .def(
            "__iter__",
            [](ProblemData::ClientGroup const &group)
            { return py::make_iterator(group.begin(), group.end()); },  // 绑定迭代器
            py::return_value_policy::reference_internal)
        .def(
            "__str__",
            [](ProblemData::ClientGroup const &group) { return group.name; },
            py::return_value_policy::reference_internal);

    py::class_<ProblemData::VehicleType>(
        m, "VehicleType", DOC(pyvrp, ProblemData, VehicleType))  // 绑定ProblemData::VehicleType类
        .def(py::init<size_t,
                      std::vector<pyvrp::Load>,
                      size_t,
                      size_t,
                      pyvrp::Cost,
                      pyvrp::Duration,
                      pyvrp::Duration,
                      pyvrp::Duration,
                      pyvrp::Distance,
                      pyvrp::Cost,
                      pyvrp::Cost,
                      size_t,
                      std::optional<pyvrp::Duration>,
                      std::vector<pyvrp::Load>,
                      std::vector<size_t>,
                      size_t,
                      pyvrp::Duration,
                      pyvrp::Cost,
                      char const *>(),
             py::arg("num_available") = 1,
             py::arg("capacity") = py::list(),
             py::arg("start_depot") = 0,
             py::arg("end_depot") = 0,
             py::arg("fixed_cost") = 0,
             py::arg("tw_early") = 0,
             py::arg("tw_late") = std::numeric_limits<pyvrp::Duration>::max(),
             py::arg("shift_duration")
             = std::numeric_limits<pyvrp::Duration>::max(),
             py::arg("max_distance")
             = std::numeric_limits<pyvrp::Distance>::max(),
             py::arg("unit_distance_cost") = 1,
             py::arg("unit_duration_cost") = 0,
             py::arg("profile") = 0,
             py::arg("start_late") = py::none(),
             py::arg("initial_load") = py::list(),
             py::arg("reload_depots") = py::list(),
             py::arg("max_reloads") = std::numeric_limits<size_t>::max(),
             py::arg("max_overtime") = 0,
             py::arg("unit_overtime_cost") = 0,
             py::kw_only(),
             py::arg("name") = "")
        .def_readonly("num_available", &ProblemData::VehicleType::numAvailable)
        .def_readonly("capacity",
                      &ProblemData::VehicleType::capacity,
                      py::return_value_policy::reference_internal)
        .def_readonly("start_depot", &ProblemData::VehicleType::startDepot)
        .def_readonly("end_depot", &ProblemData::VehicleType::endDepot)
        .def_readonly("fixed_cost", &ProblemData::VehicleType::fixedCost)
        .def_readonly("tw_early", &ProblemData::VehicleType::twEarly)
        .def_readonly("tw_late", &ProblemData::VehicleType::twLate)
        .def_readonly("shift_duration",
                      &ProblemData::VehicleType::shiftDuration)
        .def_readonly("max_distance", &ProblemData::VehicleType::maxDistance)
        .def_readonly("unit_distance_cost",
                      &ProblemData::VehicleType::unitDistanceCost)
        .def_readonly("unit_duration_cost",
                      &ProblemData::VehicleType::unitDurationCost)
        .def_readonly("profile", &ProblemData::VehicleType::profile)
        .def_readonly("start_late", &ProblemData::VehicleType::startLate)
        .def_readonly("initial_load",
                      &ProblemData::VehicleType::initialLoad,
                      py::return_value_policy::reference_internal)
        .def_readonly("reload_depots",
                      &ProblemData::VehicleType::reloadDepots,
                      py::return_value_policy::reference_internal)
        .def_readonly("max_reloads", &ProblemData::VehicleType::maxReloads)
        .def_readonly("max_overtime", &ProblemData::VehicleType::maxOvertime)
        .def_readonly("unit_overtime_cost",
                      &ProblemData::VehicleType::unitOvertimeCost)
        .def_readonly("max_duration", &ProblemData::VehicleType::maxDuration)
        .def_property_readonly("max_trips", &ProblemData::VehicleType::maxTrips)  // 计算属性max_trips
        .def_readonly("name",
                      &ProblemData::VehicleType::name,
                      py::return_value_policy::reference_internal)
        .def("replace",
             &ProblemData::VehicleType::replace,  // 绑定replace方法，用于创建修改后的副本
             py::arg("num_available") = py::none(),
             py::arg("capacity") = py::none(),
             py::arg("start_depot") = py::none(),
             py::arg("end_depot") = py::none(),
             py::arg("fixed_cost") = py::none(),
             py::arg("tw_early") = py::none(),
             py::arg("tw_late") = py::none(),
             py::arg("shift_duration") = py::none(),
             py::arg("max_distance") = py::none(),
             py::arg("unit_distance_cost") = py::none(),
             py::arg("unit_duration_cost") = py::none(),
             py::arg("profile") = py::none(),
             py::arg("start_late") = py::none(),
             py::arg("initial_load") = py::none(),
             py::arg("reload_depots") = py::none(),
             py::arg("max_reloads") = py::none(),
             py::arg("max_overtime") = py::none(),
             py::arg("unit_overtime_cost") = py::none(),
             py::kw_only(),
             py::arg("name") = py::none(),
             DOC(pyvrp, ProblemData, VehicleType, replace))
        .def(py::self == py::self)  // this is __eq__
        .def(py::pickle(
            [](ProblemData::VehicleType const &vehicleType) {  // __getstate__
                return py::make_tuple(vehicleType.numAvailable,
                                      vehicleType.capacity,
                                      vehicleType.startDepot,
                                      vehicleType.endDepot,
                                      vehicleType.fixedCost,
                                      vehicleType.twEarly,
                                      vehicleType.twLate,
                                      vehicleType.shiftDuration,
                                      vehicleType.maxDistance,
                                      vehicleType.unitDistanceCost,
                                      vehicleType.unitDurationCost,
                                      vehicleType.profile,
                                      vehicleType.startLate,
                                      vehicleType.initialLoad,
                                      vehicleType.reloadDepots,
                                      vehicleType.maxReloads,
                                      vehicleType.maxOvertime,
                                      vehicleType.unitOvertimeCost,
                                      vehicleType.name);
            },
            [](py::tuple t) {  // __setstate__
                ProblemData::VehicleType vehicleType(
                    t[0].cast<size_t>(),                    // num available
                    t[1].cast<std::vector<pyvrp::Load>>(),  // capacity
                    t[2].cast<size_t>(),                    // start depot
                    t[3].cast<size_t>(),                    // end depot
                    t[4].cast<pyvrp::Cost>(),               // fixed cost
                    t[5].cast<pyvrp::Duration>(),           // tw early
                    t[6].cast<pyvrp::Duration>(),           // tw late
                    t[7].cast<pyvrp::Duration>(),           // shift duration
                    t[8].cast<pyvrp::Distance>(),           // max distance
                    t[9].cast<pyvrp::Cost>(),       // unit distance cost
                    t[10].cast<pyvrp::Cost>(),      // unit duration cost
                    t[11].cast<size_t>(),           // profile
                    t[12].cast<pyvrp::Duration>(),  // start late
                    t[13].cast<std::vector<pyvrp::Load>>(),  // initial load
                    t[14].cast<std::vector<size_t>>(),       // reload depots
                    t[15].cast<size_t>(),                    // max reloads
                    t[16].cast<pyvrp::Duration>(),           // max overtime
                    t[17].cast<pyvrp::Cost>(),   // unit overtime cost
                    t[18].cast<std::string>());  // name

                return vehicleType;
            }))
        .def(
            "__str__",
            [](ProblemData::VehicleType const &vehType)
            { return vehType.name; },
            py::return_value_policy::reference_internal);

    py::class_<ProblemData>(m, "ProblemData", DOC(pyvrp, ProblemData))  // 绑定ProblemData类
        .def(py::init<std::vector<ProblemData::Client>,
                      std::vector<ProblemData::Depot>,
                      std::vector<ProblemData::VehicleType>,
                      std::vector<Matrix<pyvrp::Distance>>,
                      std::vector<Matrix<pyvrp::Duration>>,
                      std::vector<ProblemData::ClientGroup>>(),
             py::arg("clients"),
             py::arg("depots"),
             py::arg("vehicle_types"),
             py::arg("distance_matrices"),
             py::arg("duration_matrices"),
             py::arg("groups") = py::list())
        .def("replace",
             &ProblemData::replace,  // 绑定replace方法
             py::arg("clients") = py::none(),
             py::arg("depots") = py::none(),
             py::arg("vehicle_types") = py::none(),
             py::arg("distance_matrices") = py::none(),
             py::arg("duration_matrices") = py::none(),
             py::arg("groups") = py::none(),
             DOC(pyvrp, ProblemData, replace))
        .def_property_readonly("num_clients",
                               &ProblemData::numClients,
                               DOC(pyvrp, ProblemData, numClients))  // 只读属性num_clients
        .def_property_readonly("num_depots",
                               &ProblemData::numDepots,
                               DOC(pyvrp, ProblemData, numDepots))
        .def_property_readonly("num_groups",
                               &ProblemData::numGroups,
                               DOC(pyvrp, ProblemData, numGroups))
        .def_property_readonly("num_locations",
                               &ProblemData::numLocations,
                               DOC(pyvrp, ProblemData, numLocations))
        .def_property_readonly("num_vehicle_types",
                               &ProblemData::numVehicleTypes,
                               DOC(pyvrp, ProblemData, numVehicleTypes))
        .def_property_readonly("num_vehicles",
                               &ProblemData::numVehicles,
                               DOC(pyvrp, ProblemData, numVehicles))
        .def_property_readonly("num_profiles",
                               &ProblemData::numProfiles,
                               DOC(pyvrp, ProblemData, numProfiles))
        .def_property_readonly("num_load_dimensions",
                               &ProblemData::numLoadDimensions,
                               DOC(pyvrp, ProblemData, numLoadDimensions))
        .def(
            "location",
            [](ProblemData const &data,
               size_t idx) -> std::variant<ProblemData::Client const *,
                                           ProblemData::Depot const *>
            {
                if (idx >= data.numLocations())  // 边界检查
                    throw py::index_error();

                auto const proxy = data.location(idx);
                if (idx < data.numDepots())  // 根据索引判断是Depot还是Client
                    return proxy.depot;
                else
                    return proxy.client;
            },
            py::arg("idx"),
            py::return_value_policy::reference_internal,
            DOC(pyvrp, ProblemData, location))  // location方法，返回位置（可能是Client或Depot）
        .def("clients",
             &ProblemData::clients,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, ProblemData, clients))
        .def("depots",
             &ProblemData::depots,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, ProblemData, depots))
        .def("groups",
             &ProblemData::groups,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, ProblemData, groups))
        .def("vehicle_types",
             &ProblemData::vehicleTypes,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, ProblemData, vehicleTypes))
        .def("distance_matrices",
             &ProblemData::distanceMatrices,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, ProblemData, distanceMatrices))
        .def("duration_matrices",
             &ProblemData::durationMatrices,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, ProblemData, durationMatrices))
        .def("centroid",
             &ProblemData::centroid,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, ProblemData, centroid))
        .def("group",
             &ProblemData::group,
             py::arg("group"),
             py::return_value_policy::reference_internal,
             DOC(pyvrp, ProblemData, group))
        .def("vehicle_type",
             &ProblemData::vehicleType,
             py::arg("vehicle_type"),
             py::return_value_policy::reference_internal,
             DOC(pyvrp, ProblemData, vehicleType))
        .def("distance_matrix",
             &ProblemData::distanceMatrix,
             py::arg("profile"),
             py::return_value_policy::reference_internal,
             DOC(pyvrp, ProblemData, distanceMatrix))
        .def("duration_matrix",
             &ProblemData::durationMatrix,
             py::arg("profile"),
             py::return_value_policy::reference_internal,
             DOC(pyvrp, ProblemData, durationMatrix))
        .def("has_time_windows",
             &ProblemData::hasTimeWindows,
             DOC(pyvrp, ProblemData, hasTimeWindows))  // 检查是否有时间窗口约束
        .def(py::self == py::self)  // this is __eq__
        .def(py::pickle(
            [](ProblemData const &data) {  // __getstate__
                return py::make_tuple(data.clients(),
                                      data.depots(),
                                      data.vehicleTypes(),
                                      data.distanceMatrices(),
                                      data.durationMatrices(),
                                      data.groups());
            },
            [](py::tuple t) {  // __setstate__
                using Clients = std::vector<ProblemData::Client>;
                using Depots = std::vector<ProblemData::Depot>;
                using VehicleTypes = std::vector<ProblemData::VehicleType>;
                using DistMats = std::vector<pyvrp::Matrix<pyvrp::Distance>>;
                using DurMats = std::vector<pyvrp::Matrix<pyvrp::Duration>>;
                using Groups = std::vector<ProblemData::ClientGroup>;

                ProblemData data(t[0].cast<Clients>(),
                                 t[1].cast<Depots>(),
                                 t[2].cast<VehicleTypes>(),
                                 t[3].cast<DistMats>(),
                                 t[4].cast<DurMats>(),
                                 t[5].cast<Groups>());

                return data;
            }));

    py::class_<Trip>(m, "Trip", DOC(pyvrp, Trip))  // 绑定Trip类
        .def(py::init<ProblemData const &,
                      std::vector<size_t>,
                      size_t,
                      std::optional<size_t>,
                      std::optional<size_t>>(),
             py::arg("data"),
             py::arg("visits"),
             py::arg("vehicle_type"),
             py::arg("start_depot") = py::none(),
             py::arg("end_depot") = py::none())
        .def("visits",
             &Trip::visits,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, Trip, visits))
        .def("distance", &Trip::distance, DOC(pyvrp, Trip, distance))
        .def("delivery",
             &Trip::delivery,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, Trip, delivery))
        .def("pickup",
             &Trip::pickup,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, Trip, pickup))
        .def("load",
             &Trip::load,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, Trip, load))
        .def("excess_load",
             &Trip::excessLoad,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, Trip, excessLoad))
        .def("travel_duration",
             &Trip::travelDuration,
             DOC(pyvrp, Trip, travelDuration))
        .def("service_duration",
             &Trip::serviceDuration,
             DOC(pyvrp, Trip, serviceDuration))
        .def("release_time", &Trip::releaseTime, DOC(pyvrp, Trip, releaseTime))
        .def("prizes", &Trip::prizes, DOC(pyvrp, Trip, prizes))
        .def("centroid", &Trip::centroid, DOC(pyvrp, Trip, centroid))
        .def("vehicle_type", &Trip::vehicleType, DOC(pyvrp, Trip, vehicleType))
        .def("start_depot", &Trip::startDepot, DOC(pyvrp, Trip, startDepot))
        .def("end_depot", &Trip::endDepot, DOC(pyvrp, Trip, endDepot))
        .def("has_excess_load",
             &Trip::hasExcessLoad,
             DOC(pyvrp, Trip, hasExcessLoad))
        .def(py::self == py::self)  // this is __eq__
        .def("__len__", &Trip::size, DOC(pyvrp, Trip, size))
        .def(
            "__iter__",
            [](Trip const &trip)
            { return py::make_iterator(trip.begin(), trip.end()); },
            py::return_value_policy::reference_internal)
        .def(
            "__getitem__",
            [](Trip const &trip, int idx)
            {
                // int so we also support negative offsets from the end.
                idx = idx < 0 ? trip.size() + idx : idx;  // 支持负索引
                if (idx < 0 || static_cast<size_t>(idx) >= trip.size())
                    throw py::index_error();
                return trip[idx];
            },
            py::arg("idx"))
        .def(py::pickle(
            [](Trip const &trip) {  // __getstate__
                // Returns a tuple that completely encodes the trip's state.
                return py::make_tuple(trip.visits(),
                                      trip.distance(),
                                      trip.delivery(),
                                      trip.pickup(),
                                      trip.load(),
                                      trip.excessLoad(),
                                      trip.travelDuration(),
                                      trip.serviceDuration(),
                                      trip.releaseTime(),
                                      trip.prizes(),
                                      trip.centroid(),
                                      trip.vehicleType(),
                                      trip.startDepot(),
                                      trip.endDepot());
            },
            [](py::tuple t) {  // __setstate__
                using Coord = pyvrp::Coordinate;
                using Centroid = std::pair<Coord, Coord>;
                using Loads = std::vector<pyvrp::Load>;

                Trip trip(t[0].cast<Trip::Visits>(),     // visits
                          t[1].cast<pyvrp::Distance>(),  // distance
                          t[2].cast<Loads>(),            // delivery
                          t[3].cast<Loads>(),            // pickup
                          t[4].cast<Loads>(),            // load
                          t[5].cast<Loads>(),            // excess load
                          t[6].cast<pyvrp::Duration>(),  // travel
                          t[7].cast<pyvrp::Duration>(),  // service
                          t[8].cast<pyvrp::Duration>(),  // release
                          t[9].cast<pyvrp::Cost>(),      // prizes
                          t[10].cast<Centroid>(),        // centroid
                          t[11].cast<size_t>(),          // vehicle type
                          t[12].cast<size_t>(),          // start depot
                          t[13].cast<size_t>());         // end depot

                return trip;
            }))
        .def("__str__",
             [](Trip const &trip)
             {
                 std::stringstream stream;
                 stream << trip;
                 return stream.str();
             });

    py::class_<Route::ScheduledVisit>(
        m, "ScheduledVisit", DOC(pyvrp, Route, ScheduledVisit))  // 绑定Route::ScheduledVisit类
        .def_readonly("location", &Route::ScheduledVisit::location)
        .def_readonly("trip", &Route::ScheduledVisit::trip)
        .def_readonly("start_service", &Route::ScheduledVisit::startService)
        .def_readonly("end_service", &Route::ScheduledVisit::endService)
        .def_readonly("wait_duration", &Route::ScheduledVisit::waitDuration)
        .def_readonly("time_warp", &Route::ScheduledVisit::timeWarp)
        .def_property_readonly("service_duration",
                               &Route::ScheduledVisit::serviceDuration)  // 计算属性service_duration
        .def(py::pickle(
            [](Route::ScheduledVisit const &visit) {  // __getstate__
                return py::make_tuple(visit.location,
                                      visit.trip,
                                      visit.startService,
                                      visit.endService,
                                      visit.waitDuration,
                                      visit.timeWarp);
            },
            [](py::tuple t) {  // __setstate__
                Route::ScheduledVisit visit(
                    t[0].cast<size_t>(),            // location
                    t[1].cast<size_t>(),            // trip
                    t[2].cast<pyvrp::Duration>(),   // start service
                    t[3].cast<pyvrp::Duration>(),   // end service
                    t[4].cast<pyvrp::Duration>(),   // wait duration
                    t[5].cast<pyvrp::Duration>());  // time warp

                return visit;
            }));

    py::class_<Route>(m, "Route", DOC(pyvrp, Route))  // 绑定Route类
        .def(py::init<ProblemData const &, std::vector<size_t>, size_t>(),
             py::arg("data"),
             py::arg("visits"),
             py::arg("vehicle_type"))
        .def(py::init<ProblemData const &, std::vector<Trip>, size_t>(),
             py::arg("data"),
             py::arg("visits"),  // name is compatible with other constructor
             py::arg("vehicle_type"))
        .def("num_trips", &Route::numTrips, DOC(pyvrp, Route, numTrips))
        .def("trips",
             &Route::trips,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, Route, trips))
        .def("trip",
             &Route::trip,
             py::arg("idx"),
             py::return_value_policy::reference_internal,
             DOC(pyvrp, Route, trip))
        .def("visits",
             &Route::visits,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, Route, visits))
        .def("distance", &Route::distance, DOC(pyvrp, Route, distance))
        .def("distance_cost",
             &Route::distanceCost,
             DOC(pyvrp, Route, distanceCost))
        .def("excess_distance",
             &Route::excessDistance,
             DOC(pyvrp, Route, excessDistance))
        .def("delivery",
             &Route::delivery,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, Route, delivery))
        .def("pickup",
             &Route::pickup,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, Route, pickup))
        .def("excess_load",
             &Route::excessLoad,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, Route, excessLoad))
        .def("duration", &Route::duration, DOC(pyvrp, Route, duration))
        .def("overtime", &Route::overtime, DOC(pyvrp, Route, overtime))
        .def("duration_cost",
             &Route::durationCost,
             DOC(pyvrp, Route, durationCost))
        .def("time_warp", &Route::timeWarp, DOC(pyvrp, Route, timeWarp))
        .def("start_time", &Route::startTime, DOC(pyvrp, Route, startTime))
        .def("end_time", &Route::endTime, DOC(pyvrp, Route, endTime))
        .def("slack", &Route::slack, DOC(pyvrp, Route, slack))
        .def("travel_duration",
             &Route::travelDuration,
             DOC(pyvrp, Route, travelDuration))
        .def("service_duration",
             &Route::serviceDuration,
             DOC(pyvrp, Route, serviceDuration))
        .def("wait_duration",
             &Route::waitDuration,
             DOC(pyvrp, Route, waitDuration))
        .def(
            "release_time", &Route::releaseTime, DOC(pyvrp, Route, releaseTime))
        .def("prizes", &Route::prizes, DOC(pyvrp, Route, prizes))
        .def("centroid", &Route::centroid, DOC(pyvrp, Route, centroid))
        .def(
            "vehicle_type", &Route::vehicleType, DOC(pyvrp, Route, vehicleType))
        .def("start_depot", &Route::startDepot, DOC(pyvrp, Route, startDepot))
        .def("end_depot", &Route::endDepot, DOC(pyvrp, Route, endDepot))
        .def("is_feasible", &Route::isFeasible, DOC(pyvrp, Route, isFeasible))
        .def("has_excess_load",
             &Route::hasExcessLoad,
             DOC(pyvrp, Route, hasExcessLoad))
        .def("has_excess_distance",
             &Route::hasExcessDistance,
             DOC(pyvrp, Route, hasExcessDistance))
        .def("has_time_warp",
             &Route::hasTimeWarp,
             DOC(pyvrp, Route, hasTimeWarp))
        .def("schedule", &Route::schedule, DOC(pyvrp, Route, schedule))  // 获取调度信息
        .def("__len__", &Route::size, DOC(pyvrp, Route, size))
        .def(
            "__iter__",
            [](Route const &route)
            { return py::make_iterator(route.begin(), route.end()); },
            py::return_value_policy::reference_internal)
        .def(
            "__getitem__",
            [](Route const &route, int idx)
            {
                // conditional so we support negative offsets from the end.
                return route[idx < 0 ? route.size() + idx : idx];  // 支持负索引
            },
            py::arg("idx"))
        .def(py::self == py::self)  // this is __eq__
        .def(py::pickle(
            [](Route const &route) {  // __getstate__
                // Returns a tuple that completely encodes the route's state.
                return py::make_tuple(route.trips(),
                                      route.distance(),
                                      route.distanceCost(),
                                      route.excessDistance(),
                                      route.delivery(),
                                      route.pickup(),
                                      route.excessLoad(),
                                      route.duration(),
                                      route.overtime(),
                                      route.durationCost(),
                                      route.timeWarp(),
                                      route.travelDuration(),
                                      route.serviceDuration(),
                                      route.startTime(),
                                      route.slack(),
                                      route.prizes(),
                                      route.centroid(),
                                      route.vehicleType(),
                                      route.startDepot(),
                                      route.endDepot(),
                                      route.schedule());
            },
            [](py::tuple t) {  // __setstate__
                using Coord = pyvrp::Coordinate;
                using Centroid = std::pair<Coord, Coord>;
                using Trips = std::vector<Trip>;
                using Schedule = std::vector<Route::ScheduledVisit>;

                Route route(
                    t[0].cast<Trips>(),                     // trips
                    t[1].cast<pyvrp::Distance>(),           // distance
                    t[2].cast<pyvrp::Cost>(),               // distance cost
                    t[3].cast<pyvrp::Distance>(),           // excess distance
                    t[4].cast<std::vector<pyvrp::Load>>(),  // delivery
                    t[5].cast<std::vector<pyvrp::Load>>(),  // pickup
                    t[6].cast<std::vector<pyvrp::Load>>(),  // excess load
                    t[7].cast<pyvrp::Duration>(),           // duration
                    t[8].cast<pyvrp::Duration>(),           // overtime
                    t[9].cast<pyvrp::Cost>(),               // duration cost
                    t[10].cast<pyvrp::Duration>(),          // time warp
                    t[11].cast<pyvrp::Duration>(),          // travel
                    t[12].cast<pyvrp::Duration>(),          // service
                    t[13].cast<pyvrp::Duration>(),          // start time
                    t[14].cast<pyvrp::Duration>(),          // slack
                    t[15].cast<pyvrp::Cost>(),              // prizes
                    t[16].cast<Centroid>(),                 // centroid
                    t[17].cast<size_t>(),                   // vehicle type
                    t[18].cast<size_t>(),                   // start depot
                    t[19].cast<size_t>(),                   // end depot
                    t[20].cast<Schedule>());                // visit schedule

                return route;
            }))
        .def("__str__",
             [](Route const &route)
             {
                 std::stringstream stream;
                 stream << route;
                 return stream.str();
             });

    py::class_<Solution, std::shared_ptr<Solution>>(
        m, "Solution", DOC(pyvrp, Solution))  // 绑定Solution类，使用shared_ptr管理
        // Since Route implements __len__ and __getitem__, it is convertible to
        // std::vector<size_t> and thus a list of Routes is a valid argument for
        // both constructors. We want to avoid using the second constructor
        // since that would lose the vehicle type associations. As pybind11
        // will use the first matching constructor we put this one first.
        // 由于Route实现了__len__和__getitem__，它可以转换为std::vector<size_t>，
        // 因此Route列表对两个构造函数都是有效参数。我们想避免使用第二个构造函数，
        // 因为那会丢失车辆类型关联。由于pybind11会使用第一个匹配的构造函数，我们把这个放在前面。
        .def(py::init<ProblemData const &, std::vector<Route>>(),
             py::arg("data"),
             py::arg("routes"))
        .def(py::init<ProblemData const &,
                      std::vector<std::vector<size_t>> const &>(),
             py::arg("data"),
             py::arg("routes"))
        .def_property_readonly_static(
            "make_random",            // this is a bit of a workaround for
            [](py::object)            // classmethods, because pybind does
            {                         // not yet support those natively.
                py::options options;  // See issue 1693 in the pybind repo.
                options.disable_function_signatures();

                return py::cpp_function(
                    [](ProblemData const &data, RandomNumberGenerator &rng)
                    { return Solution(data, rng); },
                    py::arg("data"),
                    py::arg("rng"),
                    DOC(pyvrp, Solution, Solution));  // 静态方法make_random，用于创建随机解
            })
        .def(
            "num_routes", &Solution::numRoutes, DOC(pyvrp, Solution, numRoutes))
        .def("num_trips", &Solution::numTrips, DOC(pyvrp, Solution, numTrips))
        .def("num_clients",
             &Solution::numClients,
             DOC(pyvrp, Solution, numClients))
        .def("num_missing_clients",
             &Solution::numMissingClients,
             DOC(pyvrp, Solution, numMissingClients))
        .def("routes",
             &Solution::routes,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, Solution, routes))
        .def("neighbours",
             &Solution::neighbours,
             py::return_value_policy::reference_internal,
             DOC(pyvrp, Solution, neighbours))
        .def("is_feasible",
             &Solution::isFeasible,
             DOC(pyvrp, Solution, isFeasible))
        .def("is_group_feasible",
             &Solution::isGroupFeasible,
             DOC(pyvrp, Solution, isGroupFeasible))
        .def("is_complete",
             &Solution::isComplete,
             DOC(pyvrp, Solution, isComplete))
        .def("has_excess_load",
             &Solution::hasExcessLoad,
             DOC(pyvrp, Solution, hasExcessLoad))
        .def("has_excess_distance",
             &Solution::hasExcessDistance,
             DOC(pyvrp, Solution, hasExcessDistance))
        .def("has_time_warp",
             &Solution::hasTimeWarp,
             DOC(pyvrp, Solution, hasTimeWarp))
        .def("distance", &Solution::distance, DOC(pyvrp, Solution, distance))
        .def("distance_cost",
             &Solution::distanceCost,
             DOC(pyvrp, Solution, distanceCost))
        .def("duration", &Solution::duration, DOC(pyvrp, Solution, duration))
        .def("overtime", &Solution::overtime, DOC(pyvrp, Solution, overtime))
        .def("duration_cost",
             &Solution::durationCost,
             DOC(pyvrp, Solution, durationCost))
        .def("excess_load",
             &Solution::excessLoad,
             DOC(pyvrp, Solution, excessLoad))
        .def("excess_distance",
             &Solution::excessDistance,
             DOC(pyvrp, Solution, excessDistance))
        .def("fixed_vehicle_cost",
             &Solution::fixedVehicleCost,
             DOC(pyvrp, Solution, fixedVehicleCost))
        .def("time_warp", &Solution::timeWarp, DOC(pyvrp, Solution, timeWarp))
        .def("prizes", &Solution::prizes, DOC(pyvrp, Solution, prizes))
        .def("uncollected_prizes",
             &Solution::uncollectedPrizes,
             DOC(pyvrp, Solution, uncollectedPrizes))
        .def("__copy__", [](Solution const &sol) { return Solution(sol); })  // 浅拷贝
        .def(
            "__deepcopy__",
            [](Solution const &sol, py::dict) { return Solution(sol); },  // 深拷贝
            py::arg("memo"))
        .def("__hash__",
             [](Solution const &sol) { return std::hash<Solution>()(sol); })  // 哈希函数
        .def(py::self == py::self)  // this is __eq__
        .def(py::pickle(
            [](Solution const &sol) {  // __getstate__
                // Returns a tuple that completely encodes the solution's state.
                return py::make_tuple(sol.numClients(),
                                      sol.numMissingClients(),
                                      sol.distance(),
                                      sol.distanceCost(),
                                      sol.duration(),
                                      sol.overtime(),
                                      sol.durationCost(),
                                      sol.excessDistance(),
                                      sol.excessLoad(),
                                      sol.fixedVehicleCost(),
                                      sol.prizes(),
                                      sol.uncollectedPrizes(),
                                      sol.timeWarp(),
                                      sol.isGroupFeasible(),
                                      sol.routes(),
                                      sol.neighbours());
            },
            [](py::tuple t) {  // __setstate__
                using Routes = std::vector<Route>;
                using Neighbours
                    = std::vector<std::optional<std::pair<size_t, size_t>>>;

                Solution sol(
                    t[0].cast<size_t>(),                    // num clients
                    t[1].cast<size_t>(),                    // num missing
                    t[2].cast<pyvrp::Distance>(),           // distance
                    t[3].cast<pyvrp::Cost>(),               // distance cost
                    t[4].cast<pyvrp::Duration>(),           // duration
                    t[5].cast<pyvrp::Duration>(),           // overtime
                    t[6].cast<pyvrp::Cost>(),               // duration cost
                    t[7].cast<pyvrp::Distance>(),           // excess distance
                    t[8].cast<std::vector<pyvrp::Load>>(),  // excess load
                    t[9].cast<pyvrp::Cost>(),               // fixed veh cost
                    t[10].cast<pyvrp::Cost>(),              // prizes
                    t[11].cast<pyvrp::Cost>(),              // uncollected
                    t[12].cast<pyvrp::Duration>(),          // time warp
                    t[13].cast<bool>(),                     // is group feasible
                    t[14].cast<Routes>(),                   // routes
                    t[15].cast<Neighbours>());              // neighbours

                return sol;
            }))
        .def("__str__",
             [](Solution const &sol)
             {
                 std::stringstream stream;
                 stream << sol;
                 return stream.str();
             });

    py::class_<CostEvaluator>(m, "CostEvaluator", DOC(pyvrp, CostEvaluator))  // 绑定CostEvaluator类
        .def(py::init<std::vector<double>, double, double>(),
             py::arg("load_penalties"),
             py::arg("tw_penalty"),
             py::arg("dist_penalty"))
        .def("load_penalty",
             &CostEvaluator::loadPenalty,
             py::arg("load"),
             py::arg("capacity"),
             py::arg("dimension"),
             DOC(pyvrp, CostEvaluator, loadPenalty))  // 计算负载惩罚
        .def("tw_penalty",
             &CostEvaluator::twPenalty,
             py::arg("time_warp"),
             DOC(pyvrp, CostEvaluator, twPenalty))  // 计算时间窗口惩罚
        .def("dist_penalty",
             &CostEvaluator::distPenalty,
             py::arg("distance"),
             py::arg("max_distance"),
             DOC(pyvrp, CostEvaluator, distPenalty))  // 计算距离惩罚
        .def("penalised_cost",
             &CostEvaluator::penalisedCost<Solution>,
             py::arg("solution"),
             DOC(pyvrp, CostEvaluator, penalisedCost))  // 计算惩罚后的成本
        .def("cost",
             &CostEvaluator::cost<Solution>,
             py::arg("solution"),
             DOC(pyvrp, CostEvaluator, cost));  // 计算成本

    py::class_<LoadSegment>(m, "LoadSegment", DOC(pyvrp, LoadSegment))  // 绑定LoadSegment类
        .def(py::init<pyvrp::Load, pyvrp::Load, pyvrp::Load, pyvrp::Load>(),
             py::arg("delivery"),
             py::arg("pickup"),
             py::arg("load"),
             py::arg("excess_load") = 0)
        .def("delivery",
             &LoadSegment::delivery,
             DOC(pyvrp, LoadSegment, delivery))
        .def("pickup", &LoadSegment::pickup, DOC(pyvrp, LoadSegment, pickup))
        .def("load", &LoadSegment::load, DOC(pyvrp, LoadSegment, load))
        .def("excess_load",
             &LoadSegment::excessLoad,
             py::arg("capacity"),
             DOC(pyvrp, LoadSegment, excessLoad))  // 计算超额负载
        .def("finalise",
             &LoadSegment::finalise,
             py::arg("capacity"),
             DOC(pyvrp, LoadSegment, finalise))  // 最终化负载段
        .def_static(
            "merge", &LoadSegment::merge, py::arg("first"), py::arg("second"))  // 静态方法merge
        .def("__str__",
             [](LoadSegment const &segment)
             {
                 std::stringstream stream;
                 stream << segment;
                 return stream.str();
             });

    py::class_<DurationSegment>(
        m, "DurationSegment", DOC(pyvrp, DurationSegment))  // 绑定DurationSegment类
        .def(py::init<pyvrp::Duration,
                      pyvrp::Duration,
                      pyvrp::Duration,
                      pyvrp::Duration,
                      pyvrp::Duration,
                      pyvrp::Duration,
                      pyvrp::Duration,
                      pyvrp::Duration>(),
             py::arg("duration"),
             py::arg("time_warp"),
             py::arg("start_early"),
             py::arg("start_late"),
             py::arg("release_time"),
             py::arg("cum_duration") = 0,
             py::arg("cum_time_warp") = 0,
             py::arg("prev_end_late")
             = std::numeric_limits<pyvrp::Duration>::max())
        .def("duration",
             &DurationSegment::duration,
             DOC(pyvrp, DurationSegment, duration))
        .def("finalise_back",
             &DurationSegment::finaliseBack,
             DOC(pyvrp, DurationSegment, finaliseBack))  // 向后最终化
        .def("finalise_front",
             &DurationSegment::finaliseFront,
             DOC(pyvrp, DurationSegment, finaliseFront))  // 向前最终化
        .def("start_early",
             &DurationSegment::startEarly,
             DOC(pyvrp, DurationSegment, startEarly))
        .def("start_late",
             &DurationSegment::startLate,
             DOC(pyvrp, DurationSegment, startLate))
        .def("end_early",
             &DurationSegment::endEarly,
             DOC(pyvrp, DurationSegment, endEarly))
        .def("end_late",
             &DurationSegment::endLate,
             DOC(pyvrp, DurationSegment, endLate))
        .def("prev_end_late",
             &DurationSegment::prevEndLate,
             DOC(pyvrp, DurationSegment, prevEndLate))
        .def("release_time",
             &DurationSegment::releaseTime,
             DOC(pyvrp, DurationSegment, releaseTime))
        .def("slack",
             &DurationSegment::slack,
             DOC(pyvrp, DurationSegment, slack))  // 计算松弛时间
        .def("time_warp",
             &DurationSegment::timeWarp,
             py::arg("max_duration")
             = std::numeric_limits<pyvrp::Duration>::max(),
             DOC(pyvrp, DurationSegment, timeWarp))  // 计算时间扭曲
        .def_static("merge",
                    &DurationSegment::merge,
                    py::arg("edge_duration"),
                    py::arg("first"),
                    py::arg("second"))  // 静态方法merge
        .def("__str__",
             [](DurationSegment const &segment)
             {
                 std::stringstream stream;
                 stream << segment;
                 return stream.str();
             });

    py::class_<RandomNumberGenerator>(
        m, "RandomNumberGenerator", DOC(pyvrp, RandomNumberGenerator))  // 绑定RandomNumberGenerator类
        .def(py::init<uint32_t>(), py::arg("seed"))
        .def(py::init<std::array<uint32_t, 4>>(), py::arg("state"))
        .def("min", &RandomNumberGenerator::min)  // 最小可能值
        .def("max", &RandomNumberGenerator::max)  // 最大可能值
        .def("__call__", &RandomNumberGenerator::operator())  // 调用操作符
        .def("rand", &RandomNumberGenerator::rand)  // 生成随机浮点数
        .def("randint", &RandomNumberGenerator::randint<int>, py::arg("high"))  // 生成随机整数
        .def("state", &RandomNumberGenerator::state);  // 获取状态
}
