//! Tiny keyed LRU for tile caches (bounded by entry count).

use std::collections::{HashMap, VecDeque};
use std::hash::Hash;

pub struct LruMap<K, V> {
    map: HashMap<K, V>,
    order: VecDeque<K>,
    cap: usize,
}

impl<K: Clone + Eq + Hash, V> LruMap<K, V> {
    pub fn new(cap: usize) -> Self {
        Self {
            map: HashMap::new(),
            order: VecDeque::new(),
            cap: cap.max(1),
        }
    }

    pub fn len(&self) -> usize {
        self.map.len()
    }

    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.map.is_empty()
    }

    pub fn get(&mut self, key: &K) -> Option<&V> {
        if self.map.contains_key(key) {
            self.touch(key);
            self.map.get(key)
        } else {
            None
        }
    }

    #[allow(dead_code)]
    pub fn get_mut(&mut self, key: &K) -> Option<&mut V> {
        if self.map.contains_key(key) {
            self.touch(key);
            self.map.get_mut(key)
        } else {
            None
        }
    }

    pub fn insert(&mut self, key: K, value: V) {
        if self.map.contains_key(&key) {
            self.map.insert(key.clone(), value);
            self.touch(&key);
            return;
        }
        while self.map.len() >= self.cap {
            if let Some(old) = self.order.pop_front() {
                self.map.remove(&old);
            } else {
                break;
            }
        }
        self.order.push_back(key.clone());
        self.map.insert(key, value);
    }

    #[allow(dead_code)]
    pub fn contains_key(&self, key: &K) -> bool {
        self.map.contains_key(key)
    }

    fn touch(&mut self, key: &K) {
        if let Some(i) = self.order.iter().position(|k| k == key) {
            self.order.remove(i);
            self.order.push_back(key.clone());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn evicts_oldest() {
        let mut m = LruMap::new(2);
        m.insert(1, "a");
        m.insert(2, "b");
        assert!(m.get(&1).is_some()); // touch 1 → order 2,1
        m.insert(3, "c"); // evict 2
        assert!(m.get(&2).is_none());
        assert!(m.get(&1).is_some());
        assert!(m.get(&3).is_some());
    }
}
