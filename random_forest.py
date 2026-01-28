import math
import random
from collections import Counter


class DecisionTreeNode:
    def __init__(self, feature_index=None, threshold=None, left=None, right=None, value=None):
        self.feature_index = feature_index
        self.threshold = threshold
        self.left = left
        self.right = right
        self.value = value


class DecisionTree:
    def __init__(self, max_depth=10, min_samples_split=2, max_features=None):
        self.max_depth = max_depth
        self.min_samples_split = min_samples_split
        self.max_features = max_features
        self.root = None

    def fit(self, features, labels):
        self.root = self._build_tree(features, labels, depth=0)

    def predict(self, features):
        return [self._traverse_tree(row, self.root) for row in features]

    def _build_tree(self, features, labels, depth):
        num_samples = len(labels)
        num_labels = len(set(labels))

        if (
            depth >= self.max_depth
            or num_labels == 1
            or num_samples < self.min_samples_split
        ):
            return DecisionTreeNode(value=self._most_common_label(labels))

        num_features = len(features[0])
        feature_indices = list(range(num_features))
        if self.max_features is not None:
            if self.max_features == "sqrt":
                subset_size = max(1, int(math.sqrt(num_features)))
            else:
                subset_size = min(num_features, int(self.max_features))
            feature_indices = random.sample(feature_indices, subset_size)

        best_feature, best_threshold = self._best_split(features, labels, feature_indices)
        if best_feature is None:
            return DecisionTreeNode(value=self._most_common_label(labels))

        left_indices, right_indices = self._split(features, best_feature, best_threshold)
        left_features = [features[i] for i in left_indices]
        left_labels = [labels[i] for i in left_indices]
        right_features = [features[i] for i in right_indices]
        right_labels = [labels[i] for i in right_indices]

        left_node = self._build_tree(left_features, left_labels, depth + 1)
        right_node = self._build_tree(right_features, right_labels, depth + 1)
        return DecisionTreeNode(
            feature_index=best_feature,
            threshold=best_threshold,
            left=left_node,
            right=right_node,
        )

    def _best_split(self, features, labels, feature_indices):
        best_gini = 1.0
        best_feature = None
        best_threshold = None
        for feature_index in feature_indices:
            values = sorted({row[feature_index] for row in features})
            for threshold in values:
                left_indices, right_indices = self._split(features, feature_index, threshold)
                if not left_indices or not right_indices:
                    continue
                gini = self._gini_index(labels, left_indices, right_indices)
                if gini < best_gini:
                    best_gini = gini
                    best_feature = feature_index
                    best_threshold = threshold
        return best_feature, best_threshold

    def _split(self, features, feature_index, threshold):
        left_indices = []
        right_indices = []
        for index, row in enumerate(features):
            if row[feature_index] <= threshold:
                left_indices.append(index)
            else:
                right_indices.append(index)
        return left_indices, right_indices

    def _gini_index(self, labels, left_indices, right_indices):
        num_left = len(left_indices)
        num_right = len(right_indices)
        num_total = num_left + num_right
        gini_left = 1.0 - sum(
            (count / num_left) ** 2
            for count in Counter(labels[i] for i in left_indices).values()
        )
        gini_right = 1.0 - sum(
            (count / num_right) ** 2
            for count in Counter(labels[i] for i in right_indices).values()
        )
        return (num_left / num_total) * gini_left + (num_right / num_total) * gini_right

    def _most_common_label(self, labels):
        return Counter(labels).most_common(1)[0][0]

    def _traverse_tree(self, row, node):
        if node.value is not None:
            return node.value
        if row[node.feature_index] <= node.threshold:
            return self._traverse_tree(row, node.left)
        return self._traverse_tree(row, node.right)


class RandomForest:
    def __init__(self, n_trees=10, max_depth=10, min_samples_split=2, max_features="sqrt"):
        self.n_trees = n_trees
        self.max_depth = max_depth
        self.min_samples_split = min_samples_split
        self.max_features = max_features
        self.trees = []

    def fit(self, features, labels):
        self.trees = []
        for _ in range(self.n_trees):
            tree = DecisionTree(
                max_depth=self.max_depth,
                min_samples_split=self.min_samples_split,
                max_features=self.max_features,
            )
            bootstrap_features, bootstrap_labels = self._bootstrap_sample(features, labels)
            tree.fit(bootstrap_features, bootstrap_labels)
            self.trees.append(tree)

    def predict(self, features):
        tree_predictions = [tree.predict(features) for tree in self.trees]
        predictions = []
        for row_idx in range(len(features)):
            votes = [pred[row_idx] for pred in tree_predictions]
            predictions.append(Counter(votes).most_common(1)[0][0])
        return predictions

    def _bootstrap_sample(self, features, labels):
        indices = [random.randrange(len(features)) for _ in range(len(features))]
        return [features[i] for i in indices], [labels[i] for i in indices]


def accuracy_score(true_labels, pred_labels):
    correct = sum(1 for true, pred in zip(true_labels, pred_labels) if true == pred)
    return correct / len(true_labels) if true_labels else 0.0


if __name__ == "__main__":
    dataset = [
        [2.5, 1.5],
        [3.0, 1.8],
        [2.8, 1.0],
        [6.5, 2.8],
        [7.0, 3.0],
        [6.8, 2.5],
        [5.5, 2.3],
        [1.0, 0.5],
    ]
    labels = ["A", "A", "A", "B", "B", "B", "B", "A"]

    forest = RandomForest(n_trees=20, max_depth=4, min_samples_split=2, max_features="sqrt")
    forest.fit(dataset, labels)
    predictions = forest.predict(dataset)
    print("Predictions:", predictions)
    print("Accuracy:", accuracy_score(labels, predictions))
