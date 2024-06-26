import numpy as np
from lasr_vision_feature_extraction.categories_and_attributes import (
    CategoriesAndAttributes,
)


def _softmax(x: list[float]) -> list[float]:
    """Compute softmax values for each set of scores in x without using NumPy."""
    # First, compute e^x for each value in x
    exp_values = [_exp(val) for val in x]
    # Compute the sum of all e^x values
    sum_of_exp_values = sum(exp_values)
    # Now compute the softmax value for each original value in x
    softmax_values = [exp_val / sum_of_exp_values for exp_val in exp_values]
    return softmax_values


def _exp(x):
    """Compute e^x for a given x. A simple implementation of the exponential function."""
    return 2.718281828459045**x  # Using an approximation of Euler's number e


class ImageWithMasksAndAttributes:
    def __init__(
        self,
        image: np.ndarray,
        masks: dict[str, np.ndarray],
        attributes: dict[str, float],
        categories_and_attributes: CategoriesAndAttributes,
    ):
        self.image: np.ndarray = image
        self.masks: dict[str, np.ndarray] = masks
        self.attributes: dict[str, float] = attributes
        self.categories_and_attributes: CategoriesAndAttributes = (
            categories_and_attributes
        )

        self.plane_attribute_dict: dict[str, float] = {}
        for attribute in self.categories_and_attributes.plane_attributes:
            self.plane_attribute_dict[attribute] = self.attributes[attribute]

        self.selective_attribute_dict: dict[str, dict[str, float]] = {}
        for category in sorted(
            list(self.categories_and_attributes.selective_attributes.keys())
        ):
            self.selective_attribute_dict[category] = {}
            temp_list: list[float] = []
            for attribute in self.categories_and_attributes.selective_attributes[
                category
            ]:
                temp_list.append(self.attributes[attribute])
            softmax_list = _softmax(temp_list)
            for i, attribute in enumerate(
                self.categories_and_attributes.selective_attributes[category]
            ):
                self.selective_attribute_dict[category][attribute] = softmax_list[i]

    def describe(self) -> str:
        # abstract method
        pass


def _max_value_tuple(some_dict: dict[str, float]) -> tuple[str, float]:
    max_key = max(some_dict, key=some_dict.get)
    return max_key, some_dict[max_key]


class ImageOfPerson(ImageWithMasksAndAttributes):
    def __init__(
        self,
        image: np.ndarray,
        masks: dict[str, np.ndarray],
        attributes: dict[str, float],
        categories_and_attributes: CategoriesAndAttributes,
    ):
        super().__init__(image, masks, attributes, categories_and_attributes)

    @classmethod
    def from_parent_instance(
        cls, parent_instance: ImageWithMasksAndAttributes
    ) -> "ImageOfPerson":
        """
        Creates an instance of ImageOfPerson using the properties of an
        instance of ImageWithMasksAndAttributes.
        """
        return cls(
            image=parent_instance.image,
            masks=parent_instance.masks,
            attributes=parent_instance.attributes,
            categories_and_attributes=parent_instance.categories_and_attributes,
        )

    def describe(self) -> dict:
        has_hair = self.attributes["hair"] - 0.5
        hair_colour = _max_value_tuple(self.selective_attribute_dict["hair_colour"])[0]
        hair_shape = _max_value_tuple(self.selective_attribute_dict["hair_shape"])[0]
        facial_hair = 1 - self.attributes["No_Beard"] - 0.5
        glasses = self.attributes["Eyeglasses"] - 0.5
        hat = self.attributes["Wearing_Hat"] - 0.5

        result = {
            "has_hair": has_hair,
            "hair_colour": hair_colour,
            "hair_shape": hair_shape,
            "facial_hair": facial_hair,
            "glasses": glasses,
            "hat": hat,
        }

        return result


class ImageOfCloth(ImageWithMasksAndAttributes):
    def __init__(
        self,
        image: np.ndarray,
        masks: dict[str, np.ndarray],
        attributes: dict[str, float],
        categories_and_attributes: CategoriesAndAttributes,
    ):
        super().__init__(image, masks, attributes, categories_and_attributes)

    @classmethod
    def from_parent_instance(
        cls, parent_instance: ImageWithMasksAndAttributes
    ) -> "ImageOfCloth":
        """
        Creates an instance of ImageOfCloth using the properties of an
        instance of ImageWithMasksAndAttributes.
        """
        return cls(
            image=parent_instance.image,
            masks=parent_instance.masks,
            attributes=parent_instance.attributes,
            categories_and_attributes=parent_instance.categories_and_attributes,
        )

    def describe(self) -> dict:
        result = {
            "top": self.attributes["top"] / 2,
            "outwear": self.attributes["outwear"] / 2,
            "dress": self.attributes["dress"] / 2,
        }

        max_prob = 0.0
        max_attribute = "short sleeve top"
        for attribute in ["short sleeve top", "long sleeve top", "vest", "sling"]:
            if self.attributes[attribute] > max_prob:
                max_prob = self.attributes[attribute]
                max_attribute = attribute
        if max_attribute in ["vest", "sling"]:
            max_attribute = "sleeveless top"
        result["max_top"] = max_attribute

        max_prob = 0.0
        max_attribute = "short sleeve outwear"
        for attribute in [
            "short sleeve outwear",
            "long sleeve outwear",
        ]:
            if self.attributes[attribute] > max_prob:
                max_prob = self.attributes[attribute]
                max_attribute = attribute
        result["max_outwear"] = max_attribute

        max_prob = 0.0
        max_attribute = "short sleeve dress"
        for attribute in [
            "short sleeve dress",
            "long sleeve dress",
            "vest dress",
            "sling dress",
        ]:
            if self.attributes[attribute] > max_prob:
                max_prob = self.attributes[attribute]
                max_attribute = attribute
        if max_attribute in ["vest dress", "sling dress"]:
            max_attribute = "sleeveless dress"
        result["max_dress"] = max_attribute

        return result
