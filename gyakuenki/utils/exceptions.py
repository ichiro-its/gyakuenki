class CameraInfoNotSetException(Exception):
    """Raised is a transform is requested without CameraInfo being set."""

    pass


class InvalidPlaneException(Exception):
    """Raised if a plane is invalid, i.e. a=b=c=0."""

    pass


class NoIntersectionError(Exception):
    """Raised if a point is not able to be mapped onto the plane."""

    pass
