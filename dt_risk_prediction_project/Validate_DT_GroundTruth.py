"""Validate a saved Source/Prediction bundle without CARLA or forecast scoring."""
import argparse
import json
import os
import stat
from pathlib import Path

from dt_gt_manifest import validate


def guarded_output(manifest, output):
    """Reject unsafe paths before creating anything; requires stable parent dirs.

    Exclusive create protects the leaf. This is not a sandbox against concurrent
    hostile replacement of parent directories between this check and open().
    """
    specified = output.absolute()
    # Check the uncollapsed chain too: a link followed by '..' is still a link.
    for component in (specified, *specified.parents):
        if os.path.lexists(component):
            info = component.lstat()
            if stat.S_ISLNK(info.st_mode) or getattr(info, 'st_file_attributes', 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT:
                raise ValueError('symlink/junction/reparse output path forbidden')
    normalized = Path(os.path.abspath(specified))
    resolved = specified.resolve()
    repository = Path(__file__).resolve().parent.parent
    input_dirs = (Path(os.path.abspath(manifest)), manifest.resolve())
    for candidate in (normalized, resolved):
        if candidate.is_relative_to(repository):
            raise ValueError('Repository output forbidden')
        if any(candidate.is_relative_to(p.parent) for p in input_dirs):
            raise ValueError('input directory output forbidden')
        if any(os.path.lexists(parent / '.git') for parent in candidate.parents):
            raise ValueError('Git worktree output forbidden')
    if os.path.lexists(specified):
        raise ValueError('new output path required')
    if not specified.parent.is_dir():
        raise ValueError('existing external output directory required')
    return normalized


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    try:
        output=guarded_output(args.manifest,args.output)
    except (OSError,ValueError,RuntimeError) as error:
        parser.error(str(error))
    result=validate(args.manifest)
    # Recheck after validation; never overwrite or select a fallback path.
    try:
        if guarded_output(args.manifest,args.output)!=output:
            raise ValueError('output path changed during validation')
        with output.open('x',encoding='utf-8') as stream:
            json.dump(result,stream,indent=2,allow_nan=False)
    except (OSError,ValueError,RuntimeError) as error:
        parser.error(str(error))
    print(json.dumps({k:result[k] for k in ('status','errors','independent_forecast_accuracy_enabled')}))
    return 2 if result['errors'] else 0


if __name__=='__main__':raise SystemExit(main())
