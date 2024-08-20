use std::{cmp::Ordering, collections::BinaryHeap, sync::{Arc, RwLock}};
use hashbrown::{HashMap, HashSet};
use indexmap::IndexMap;

use bevy::prelude::*;
use thiserror::Error;

use std::hash::{Hash, Hasher};
use rustc_hash::FxHasher;
use std::hash::BuildHasherDefault;

use indexmap::map::Entry::{Occupied, Vacant};

type FxIndexMap<K, V> = IndexMap<K, V, BuildHasherDefault<FxHasher>>;

#[derive(Default, Debug, Clone)]
struct PointGuard {
    pub id: i64,
    pub point: Arc<RwLock<Point>>,
}

impl PointGuard {
    fn new(id: i64, pos: Vec3, weight: f32, enabled: bool) -> Self {
        PointGuard {
            id,
            point: Arc::new(RwLock::new(Point {
                pos,
                weight,
                enabled,
                ..Default::default()
            }))
        }
    }
}

impl Eq for PointGuard{}

impl PartialEq for PointGuard {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Hash for PointGuard {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

/*impl PartialOrd for PointGuard {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        other.0.read().unwrap().f_score.partial_cmp(&self.0.read().unwrap().f_score)
    }
}

impl Ord for PointGuard {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap().reverse()
    }
} */

#[derive(Default, Debug, Clone)]
struct Point {
    pos: Vec3,

    weight: f32,
    enabled: bool,

    // Neighbors
    neighbors: HashSet<PointGuard>,
    unlinked_neighbors: HashSet<PointGuard>,

    /*prev_point: Option<PointGuard>,

    open_pass: u64,
    closed_pass: u64,

    g_score: f32,
    f_score: f32,

    // Used for getting closest_point_of_last_pathing_call.
    abs_g_score: f32,
    abs_f_score: f32,*/
}
/*
impl PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}

impl Eq for Point {}

impl PartialOrd for Point {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        other.f_score.partial_cmp(&self.f_score)
    }
}

impl Ord for Point {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap().reverse()
    }
} */

/* mod segment_direction {
    pub const NONE: u8 = 0;
    pub const FORWARD: u8 = 1;
    pub const BACKWARD: u8 = 2;
    pub const BIDIRECTIONAL: u8 = FORWARD | BACKWARD;
}

#[derive(Eq, Hash, PartialEq)]
struct Segment {
    key: (i64, i64),
    direction: u8,
} */

#[derive(Default, Resource, Clone)]
pub struct Pathfinding {
    points: HashMap<i64, PointGuard>,
    //segments: HashSet<Segment>,

    pass: u64,
    last_closest_point: Option<PointGuard>,
}

// Errors
#[derive(Debug, Clone, Error)]
pub enum PathfindingError {
    #[error("Point does not exist")]
    PointDoesNotExist,
    #[error("IDs are equal")]
    IdsAreEqual,
    #[error("Bad index")]
    BadIndex,
    #[error("Path is unsolvable")]
    Unsolvable,
    #[error("Lib error")]
    LibError,
}

struct SmallestCostHolder {
    estimated_cost: f32,
    cost: f32,
    index: usize,
}

impl PartialEq for SmallestCostHolder {
    fn eq(&self, other: &Self) -> bool {
        self.estimated_cost.eq(&other.estimated_cost) && self.cost.eq(&other.cost)
    }
}

impl Eq for SmallestCostHolder {}

impl PartialOrd for SmallestCostHolder {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for SmallestCostHolder {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        match self.estimated_cost.to_bits().cmp(&other.estimated_cost.to_bits()) {
            Ordering::Equal => {self.cost.to_bits().cmp(&other.cost.to_bits())},
            s => s,
        }
    }
}

/*impl Hash for SmallestCostHolder {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}*/


impl Pathfinding {
    pub fn add_vec(&mut self, pos: Vec3, weight: f32) {
        let index = Pathfinding::calculate_index(pos);
        self.add_point(index, pos, weight);
    }

    pub fn add_point(&mut self, id: i64, pos: Vec3, weight: f32) {
        if let Some(point_guard) = self.points.get_mut(&id) {
            let mut point = point_guard.point.write().unwrap();
            point.pos = pos;
            point.weight = weight;
        } else {
            self.points.insert(id, PointGuard::new(id, pos, weight, true));
        }
    }


    fn get_point(&self, id: &i64) -> Result<&PointGuard, PathfindingError> {
        let Some(point) = self.points.get(id) else {return Err(PathfindingError::PointDoesNotExist)};

        Ok(point)
    }

    pub fn connect_points(&mut self, from: Vec3, to: Vec3, bidirectional: bool) -> Result<(), PathfindingError> {
        let from_index = Pathfinding::calculate_index(from);
        let to_index = Pathfinding::calculate_index(to);

        self.connect_points_by_index(from_index, to_index, bidirectional)
    }

    pub fn connect_points_by_index(&mut self, from_id: i64, to_id: i64, bidirectional: bool) -> Result<(), PathfindingError> {
        if from_id == to_id {
            return Err(PathfindingError::IdsAreEqual);
        }

        let Some(from_guard) = self.points.get(&from_id) else {return Err(PathfindingError::PointDoesNotExist)};
        let Some(to_guard) = self.points.get(&to_id) else {return Err(PathfindingError::PointDoesNotExist)};

        let mut from = from_guard.point.write().unwrap();
        from.neighbors.insert(to_guard.clone());

        std::mem::drop(from);

        if bidirectional {
            let mut to = to_guard.point.write().unwrap();
            to.neighbors.insert(from_guard.clone());
        } else {
            let mut to = to_guard.point.write().unwrap();
            to.unlinked_neighbors.insert(from_guard.clone());
        }

        /* let mut s = Segment { key: (from_id, to_id), direction: segment_direction::NONE };

        if bidirectional {
            s.direction = segment_direction::BIDIRECTIONAL;
        }

        if let Some(element) = self.segments.get(&s) {
            s.direction |= element.direction;

            if s.direction == segment_direction::BIDIRECTIONAL {
                to.unlinked_neighbors.remove(from_guard);
                from.unlinked_neighbors.remove(to_guard);
            }
        }

        self.segments.insert(s);*/

        Ok(())
    }

    pub fn calculate_index(pos: Vec3) -> i64 {
        let max_value: i64 = (1 << 20) - 1; // Maximum value that can be packed in 20 bits.
        let x = (pos.x as i64) & max_value;
        let y = (pos.y as i64) & max_value;
        let z = (pos.z as i64) & max_value;
    
        (x << 40) | (y << 20) | z
    }


    /*pub fn get_point_path_by_index(
        &mut self, 
        start_index: i64, 
        end_index: i64,
        allow_partial: bool
    ) -> Result<Vec<Vec3>, PathfindingError> {
        let mut path: Vec<Vec3> = Vec::new();

        if !self.points.contains_key(&start_index) || !self.points.contains_key(&end_index) {
            return Err(PathfindingError::BadIndex)
        }

        if start_index == end_index {
            return Err(PathfindingError::IdsAreEqual)
        }

        if !self.solve(start_index, end_index, allow_partial)? {
            return Err(PathfindingError::Unsolvable)
        }

        let mut current = self.last_closest_point.clone();

        while let Some(point_guard) = current {
            let point = point_guard.0.read().unwrap();
            path.push(point.pos);
            current = point.prev_point.clone();
        }

        path.reverse();

        Ok(path)
    }*/

    /*pub fn get_point_path(
        &mut self, 
        start: Vec3, 
        end: Vec3, 
        allow_partial: bool
    ) -> Result<Vec<Vec3>, PathfindingError> {
        let start_index = Pathfinding::calculate_index(start);
        let end_index = Pathfinding::calculate_index(end);

        self.get_point_path_by_index(start_index, end_index, allow_partial)
    }*/

    pub fn get_path_by_vec(&self, begin: Vec3, end: Vec3, allow_partial: bool) -> Result<Vec<Vec3>, PathfindingError> {
        let begin_index = Pathfinding::calculate_index(begin);
        let end_index = Pathfinding::calculate_index(end);

        self.get_path(begin_index, end_index, allow_partial)
    }

    pub fn get_path(&self, begin_index: i64, end_index: i64, allow_partial: bool) -> Result<Vec<Vec3>, PathfindingError> {
        if begin_index == end_index {
            return Err(PathfindingError::IdsAreEqual)
        }

        let begin_guard = self.get_point(&begin_index)?;
        let end_guard = self.get_point(&end_index)?;

        let end_id = end_guard.id;

        let mut to_see = BinaryHeap::new();
        to_see.push(SmallestCostHolder {
            estimated_cost: 0.0,
            cost: 0.0,
            index: 0,
        });

        let mut parents: FxIndexMap<PointGuard, (usize, f32)> = FxIndexMap::default();
        parents.insert(begin_guard.clone(), (usize::MAX, 0.0));

        while let Some(SmallestCostHolder{ cost, index, .. }) = to_see.pop() {
            let (point_guard, &(_, c)) = parents.get_index(index).unwrap();
            let point_guard = point_guard.clone();
            let point = point_guard.point.read().unwrap();
            
            if point_guard.id == end_id {
                let mut i = index;
                let mut path = std::iter::from_fn(|| {
                    parents.get_index(i).map(|(guard, value)| {
                        i = value.0;
                        guard.point.read().unwrap().pos
                    })
                }).collect::<Vec<Vec3>>();
                path.reverse();
                return Ok(path)

                //parents.get(&point_guard).map(|| )
            }

            if cost > c {
                continue;
            }

            for neighbor_guard in point.neighbors.clone().into_iter() {
                let neighbor = neighbor_guard.point.read().unwrap();

                let new_cost = cost + neighbor.weight;
                let new_index: usize;

                match parents.entry(neighbor_guard.clone()) {
                    Vacant(e) => {
                        new_index = e.index();
                        e.insert((index, new_cost));
                    },
                    Occupied(mut e) => {
                        if e.get().1 > new_cost {
                            new_index = e.index();
                            e.insert((index, new_cost));
                        } else {
                            continue;
                        }
                    },
                }

                to_see.push(SmallestCostHolder{
                    estimated_cost: new_cost + neighbor.pos.distance(point.pos),
                    cost: new_cost,
                    index: new_index,
                });
            }
        }

        Ok(Vec::new())
    }

    /*fn solve(&mut self, begin_index: i64, end_index: i64, allow_partial: bool) -> Result<bool, PathfindingError> {
        if begin_index == end_index {
            return Ok(false);
        }

        let begin_guard = self.get_point(&begin_index)?;
        let end_guard = self.get_point(&end_index)?;

        let begin = begin_guard.0.read().unwrap();
        let end_pos = {
            let end = end_guard.0.read().unwrap();
            end.pos
        };

        let mut open: IndexMap<i64, SmallestCostHolder> = IndexMap::new();
        let mut closed: HashSet<i64> = HashSet::new();

        let begin_smallest_cost = SmallestCostHolder {
            estimated_cost: begin.pos.distance(end_pos),
            cost: begin.pos.distance(end_pos),
            point_guard: begin_guard.clone(),
        };

        open.insert(begin.id, begin_smallest_cost);
        closed.insert(begin.id);

        std::mem::drop(begin);

        while !open.is_empty() {
            let (point_id, smallest_cost) = open.pop().unwrap();
            let point = smallest_cost.point.0.read().unwrap();

            if point.id == end_index {
                break;
            }

            for neighbor_guard in point.neighbors.clone().into_iter() {
                let neighbor = neighbor_guard.0.read().unwrap();

                if !neighbor.enabled || closed.contains(&neighbor.id) {
                    continue;
                }

                let distance = point.pos.distance(neighbor.pos);

                let estimated_cost = smallest_cost.cost + distance * neighbor.weight;

                let mut neighbor_smallest_cost: &SmallestCostHolder;

                let maybe_open = open.get_mut(&neighbor.id);

                if let Some(open) = maybe_open {
                    if estimated_cost >= open.cost {
                        continue;
                    } else {
                        neighbor_smallest_cost = open;
                    }
                } else {
                    neigb
                }

                neighbor.prev_point = Some(point_guard.clone()); 
                neighbor.g_score = tentative_g_score;
                neighbor.f_score = neighbor.g_score + distance;
                neighbor.abs_g_score = tentative_g_score;
                neighbor.abs_f_score = neighbor.f_score - neighbor.g_score;

                std::mem::drop(neighbor);                               

                if new_point { open_list.push(neighbor_guard.clone()) };

            }

            std::mem::drop(point);
        }

        if let Some(lcp) = &self.last_closest_point {
            if end_index == lcp.0.read().unwrap().id || allow_partial {
                return Ok(true);
            } else {
                return Ok(false);
            }
        } else {
            Err(PathfindingError::Unsolvable)
        }
    }*/
}

#[cfg(test)]
mod tests {
    use std::collections::BinaryHeap;
    use std::sync::Arc;
    use std::sync::RwLock;

    use crate::FxIndexMap;
    use crate::Pathfinding;
    use crate::PathfindingError;
    use bevy::math::Vec3;
    use crate::Point;
    use crate::PointGuard;

    #[test]
    fn corner_to_corner() {
        let mut pathfinding = Pathfinding::default();

        for x in 0..16 {
            for y in 0..16 {
                pathfinding.add_vec(Vec3::new(x as f32, y as f32, 0.0), 0.0);
            }
        }
    
        for x in 0..16 {
            for y in 0..16 {
                if x > 0 {
                    pathfinding.connect_points(
                        Vec3::new(x as f32, y as f32, 0.), 
                        Vec3::new(x as f32 - 1.0, y as f32, 0.), 
                        false
                    ).unwrap();
                }
                if x < 15 {
                    pathfinding.connect_points(
                        Vec3::new(x as f32, y as f32, 0.), 
                        Vec3::new(x as f32 + 1.0, y as f32, 0.), 
                        false
                    ).unwrap();
                }
                if y > 0 {
                    pathfinding.connect_points(
                        Vec3::new(x as f32, y as f32, 0.), 
                        Vec3::new(x as f32, y as f32 - 1.0, 0.), 
                        false 
                    ).unwrap();
                }
                if y < 15 {
                    pathfinding.connect_points(
                        Vec3::new(x as f32, y as f32, 0.), 
                        Vec3::new(x as f32, y as f32 + 1.0, 0.), 
                        false
                    ).unwrap();
                }
            }
        }
    
        let _ = pathfinding.get_path_by_vec(
            Vec3::new(0., 0., 0.),
            Vec3::new(15., 15., 0.),
            false
        );
    }

/*    #[test]
    fn test_point_ordering() {
        let mut a = Point::default();
        a.f_score = 1.0;
        let mut b = Point::default();
        b.f_score = 1.2;
        let mut c = Point::default();
        c.f_score = 1.4;

        let mut heap: BinaryHeap<&Point> = BinaryHeap::new();
        heap.push(&a);
        heap.push(&c);
        heap.push(&b);

        assert_eq!(a == b, false);
        assert_eq!(a < b, false);
        assert_eq!(heap.pop(), Some(&a));
    }

    #[test]
    fn test_calculate_index() {
        let idx1 = Pathfinding::calculate_index(Vec3::new(1.0, 0.0, 0.0));
        let idx2 = Pathfinding::calculate_index(Vec3::new(0.0, 1.0, 0.0));
        let idx3 = Pathfinding::calculate_index(Vec3::new(0.0, 0.0, 1.0));

        assert!(idx1 != idx2 && idx1 != idx3 && idx2 != idx3);
    }

    #[test]
    fn test_same_id() {
        let mut pathfinding = Pathfinding::default();

        pathfinding.add_point(0, Vec3::new(0.0, 0.0, 0.0), 1.0);

        assert_eq!(pathfinding.solve(0, 0, false).unwrap(), false);
    }*/

    #[test]
    fn test_something() {
        let mut map: FxIndexMap<String, (usize, f32)> = FxIndexMap::default();

        map.insert("Beta".to_string(), (usize::MAX, 1.0));
        map.insert("Gamma".to_string(), (40, 1.8));
        map.insert("Alpha".to_string(), (60, 1.4));

        println!("{:?}", map);
    }

    #[test]
    fn test_solveable() {
        let mut pathfinding = Pathfinding::default();

        let v1 = Vec3::new(0.0, 0.0, 0.0);
        let i1 = Pathfinding::calculate_index(v1);
        let v2 = Vec3::new(1.0, 0.0, 0.0);
        let i2 = Pathfinding::calculate_index(v2);
        let v3 = Vec3::new(2.0, 0.0, 0.0);
        let i3 = Pathfinding::calculate_index(v3);


        pathfinding.add_point(i1, v1, 1.0);
        pathfinding.add_point(i2, v2, 1.0);
        pathfinding.add_point(i3, v3, 1.0);

        let _ = pathfinding.connect_points_by_index(i1, i2, true);
        let _ = pathfinding.connect_points_by_index(i2, i3, true);

        let path = pathfinding.get_path(i1, i3, false);

        assert_eq!(path.unwrap(), vec![v1, v2, v3]);
    }

/*    #[test]
    fn test_unsolveable() {
        let mut pathfinding = Pathfinding::default();

        let v1 = Vec3::new(0.0, 0.0, 0.0);
        let i1 = Pathfinding::calculate_index(v1);
        let v2 = Vec3::new(1.0, 0.0, 0.0);
        let i2 = Pathfinding::calculate_index(v2);
        let v3 = Vec3::new(2.0, 0.0, 1.0);
        let i3 = Pathfinding::calculate_index(v3);
        let v4 = Vec3::new(3.0, 0.0, 0.0);
        let i4 = Pathfinding::calculate_index(v4);

        pathfinding.add_point(i1, v1, 0.0);
        pathfinding.add_point(i2, v2, 0.0);
        pathfinding.add_point(i3, v3, 0.0);
        pathfinding.add_point(i4, v4, 0.0);

        let _ = pathfinding.connect_points_by_index(i1, i2, false);
        let _ = pathfinding.connect_points_by_index(i2, i3, false);

        assert_eq!(pathfinding.solve(i1, i4, false).unwrap(), false);        
    }

    #[test]
    fn test_partial() {
        let mut pathfinding = Pathfinding::default();

        let v1 = Vec3::new(0.0, 0.0, 0.0);
        let i1 = Pathfinding::calculate_index(v1);
        let v2 = Vec3::new(1.0, 0.0, 0.0);
        let i2 = Pathfinding::calculate_index(v2);
        let v3 = Vec3::new(2.0, 0.0, 1.0);
        let i3 = Pathfinding::calculate_index(v3);
        let v4 = Vec3::new(3.0, 0.0, 0.0);
        let i4 = Pathfinding::calculate_index(v4);

        pathfinding.add_point(i1, v1, 0.0);
        pathfinding.add_point(i2, v2, 0.0);
        pathfinding.add_point(i3, v3, 0.0);
        pathfinding.add_point(i4, v4, 0.0);

        let _ = pathfinding.connect_points_by_index(i1, i2, false);
        let _ = pathfinding.connect_points_by_index(i2, i3, false);

        assert_eq!(pathfinding.solve(i1, i4, true).unwrap(), true);        
    }

    #[test]
    fn test_get_path() {
        let mut pathfinding = Pathfinding::default();

        let v1 = Vec3::new(0.0, 0.0, 0.0);
        let i1 = Pathfinding::calculate_index(v1);
        let v2 = Vec3::new(1.0, 0.0, 0.0);
        let i2 = Pathfinding::calculate_index(v2);
        let v3 = Vec3::new(2.0, 0.0, 0.0);
        let i3 = Pathfinding::calculate_index(v3);

        pathfinding.add_point(i1, v1, 1.0);
        pathfinding.add_point(i2, v2, 1.0);
        pathfinding.add_point(i3, v3, 1.0);

        let _ = pathfinding.connect_points_by_index(i1, i2, true);
        let _ = pathfinding.connect_points_by_index(i2, i3, true);

        let path = pathfinding.get_point_path(v1, v3, false).unwrap();

        let test = vec![v1, v2, v3];

        assert_eq!(path, test);
    }

    #[test]
    fn test_weighting() {
        let path_one: Vec<Vec3>;
        let path_two: Vec<Vec3>;

        let mut pathfinding = Pathfinding::default();
        let mut grid: Vec<[(f32, i64); 4]> = vec![
            [(0.0, 0), (0.0, 1), (0.0, 2), (0.0, 3)],
            [(255.0, 4), (0.0, 5), (0.0, 6), (0.0, 7)],
            [(0.0, 8), (0.0, 9), (0.0, 10), (0.0, 11)],
            [(0.0, 12), (0.0, 13), (0.0, 14), (0.0, 15)],
        ];

        for x in 0..4 {
            for y in 0..4 {
                let vec = Vec3::new(x as f32, y as f32, 0.0);
                pathfinding.add_point(grid[x][y].1, vec, grid[x][y].0);
            }
        }

        for x in 0..4 {
            for y in 0..4 {
                if x > 0 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x-1][y].1, false).unwrap();
                }
                if x < 3 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x+1][y].1, false).unwrap();
                }
                if y > 0 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x][y-1].1, false).unwrap();
                }
                if y < 3 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x][y+1].1, false).unwrap();
                }
            }
        }

        path_one = pathfinding.get_point_path_by_index(0, 15, false).unwrap();

        grid[1][0].0 = 0.0;

        let mut pathfinding = Pathfinding::default();

        for x in 0..4 {
            for y in 0..4 {
                let vec = Vec3::new(x as f32, y as f32, 0.0);
                pathfinding.add_point(grid[x][y].1, vec, grid[x][y].0);
            }
        }

        for x in 0..4 {
            for y in 0..4 {
                if x > 0 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x-1][y].1, false).unwrap();
                }
                if x < 3 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x+1][y].1, false).unwrap();
                }
                if y > 0 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x][y-1].1, false).unwrap();
                }
                if y < 3 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x][y+1].1, false).unwrap();
                }
            }
        }        

        path_two = pathfinding.get_point_path_by_index(0, 15, false).unwrap();

        assert_ne!(path_one, path_two);
    } */
}